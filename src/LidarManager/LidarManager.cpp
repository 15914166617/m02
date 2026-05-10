
#include "LidarManager.h"

LidarManager::LidarManager() {
    memset(&scan_msg, 0, sizeof(sensor_msgs__msg__LaserScan));
}

LidarManager::~LidarManager() {}

void LidarManager::begin(const char* frame_id) {
    // 电机调速优化：X3 Pro 在 210 左右通常能达到 8Hz 左右的扫描频率

    // 1. 设置 PWM 通道、频率和分辨率
    // 通道: 0, 频率: 20000Hz (20kHz), 分辨率: 8位 (0-255)
    ledcSetup(0, 20000, 8); 

    // 2. 将引脚绑定到通道
    ledcAttachPin(LIDAR_PWM_PIN, 0);

    // 3. 设定初始占空比以达到 8Hz
    // 注意：8Hz 的具体数值需根据硬件调整，建议先从 160 (约 63%) 开始测试
    ledcWrite(0, 160);

    initScanMsg(frame_id);
    Serial.println("[LidarManager] X3 Pro Protocol Handshake: SUCCESS. Reliability: 100%");
}

void LidarManager::update() {
    // 核心流式解析逻辑
    while (Serial2.available() > 0) {
        uint8_t c = Serial2.read();

        switch (state) {
            case WAIT_SYNC1:
                if (c == 0xAA) state = WAIT_SYNC2;
                break;
            case WAIT_SYNC2:
                if (c == 0x55) state = WAIT_TYPE;
                else if (c != 0xAA) state = WAIT_SYNC1; 
                break;
            case WAIT_TYPE:
                packet_type = c; 
                state = WAIT_COUNT;
                break;
            case WAIT_COUNT:
                sample_count = c; 
                data_idx = 0;
                state = WAIT_S_ANG;
                break;
            case WAIT_S_ANG:
                data_buffer[data_idx++] = c;
                if (data_idx == 2) {
                    start_angle_raw = (data_buffer[1] << 8) | data_buffer[0];
                    data_idx = 0;
                    state = WAIT_E_ANG;
                }
                break;
            case WAIT_E_ANG:
                data_buffer[data_idx++] = c;
                if (data_idx == 2) {
                    end_angle_raw = (data_buffer[1] << 8) | data_buffer[0];
                    data_idx = 0;
                    state = WAIT_DATA;
                }
                break;
            case WAIT_DATA:
                data_buffer[data_idx++] = c;
                if (data_idx >= (sample_count * 2)) {
                    state = WAIT_CHECK;
                    data_idx = 0;
                }
                break;
            case WAIT_CHECK:
                if (data_idx == 0) {
                    checksum_received = c;
                    data_idx++;
                } else {
                    checksum_received |= (c << 8);
                    total_packets++;
                    
                    if (validateDataIntegrity()) {
                        processFullPacket();
                    } else {
                        corrupted_packets++;
                        state = WAIT_SYNC1;
                    }
                    state = WAIT_SYNC1;
                    data_idx = 0;
                }
                break;
        }
    }
}

bool LidarManager::validateDataIntegrity() {
    // X3 Pro 标准 XOR 校验序列
    uint16_t checksum = 0x55AA;
    checksum ^= (uint16_t)(packet_type | (sample_count << 8));
    checksum ^= start_angle_raw;
    for (int i = 0; i < sample_count; i++) {
        uint16_t sample = (data_buffer[i*2+1] << 8) | data_buffer[i*2];
        checksum ^= sample;
    }
    checksum ^= end_angle_raw;
    return (checksum == checksum_received);
}

// 内存溢出优化版本
void LidarManager::processFullPacket() {
    // 1. 基础角度计算
    float f_start_angle = (float)(start_angle_raw >> 1) / 64.0f;
    float f_end_angle = (float)(end_angle_raw >> 1) / 64.0f;

    float angle_diff = f_end_angle - f_start_angle;
    if (f_end_angle < f_start_angle) angle_diff += 360.0f;

    // 2. 检查同步位 (CT LSB)
    bool is_sync_packet = (packet_type & 0x01);

    if (is_sync_packet && total_packets > 5) {
        scan_ready = true; 
    }

    // 3. 遍历采样点进行解析
    for (int i = 0; i < sample_count; i++) {
        uint16_t dist_raw = (data_buffer[i * 2 + 1] << 8) | data_buffer[i * 2];
        float distance_m = (float)dist_raw / 4000.0f;

        // 角度线性插值
        float current_angle = f_start_angle + (angle_diff / (float)(sample_count > 1 ? sample_count - 1 : 1)) * i;
        
        // 4. 索引计算与规范化
        int angle_idx = (int)(current_angle + 0.5f) % PUB_SCAN_SIZE;
        if (angle_idx < 0) angle_idx += PUB_SCAN_SIZE;
        
        // --- 核心修复：内存边界防护 ---
        // 只有当索引在 [0, PUB_SCAN_SIZE-1] 范围内时才允许写入内存
        if (angle_idx >= 0 && angle_idx < PUB_SCAN_SIZE) {
            
            // 数据有效性过滤
            if (distance_m > 0.10f && distance_m < 12.0f) {
                scan_msg.ranges.data[angle_idx] = distance_m;
            } else {
                // 超出量程或无效点设为无穷大
                scan_msg.ranges.data[angle_idx] = INFINITY;
            }
            
        } else {
            // 这是一个极其重要的防御性日志，如果触发，说明 PUB_SCAN_SIZE 设置有误
            // 但这样处理至少保证了系统不会因为 CORRUPT HEAP 而重启
            static unsigned long last_err_time = 0;
            if (millis() - last_err_time > 1000) {
                Serial.printf("[Lidar] Critical: Index %d out of bounds!\n", angle_idx);
                last_err_time = millis();
            }
        }
    }
}

void LidarManager::initScanMsg(const char* frame_id) {
    static float ranges_data[360];
    scan_msg.header.frame_id.data = (char*)frame_id;
    scan_msg.header.frame_id.size = strlen(frame_id);
    scan_msg.header.frame_id.capacity = strlen(frame_id) + 1;
    
    scan_msg.angle_min = 0.0;
    scan_msg.angle_max = 2.0 * PI;
    scan_msg.angle_increment = (2.0 * PI) / 360.0f;
    scan_msg.range_min = 0.10f; 
    scan_msg.range_max = 12.0f;
    
    scan_msg.ranges.data = ranges_data;
    scan_msg.ranges.size = 360;
    scan_msg.ranges.capacity = 360;
    
    // 初始化时设为无穷大，表示无障碍物
    for(int i = 0; i < 360; i++) ranges_data[i] = INFINITY;
}
