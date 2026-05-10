#ifndef LIDAR_MANAGER_H
#define LIDAR_MANAGER_H

#include <Arduino.h>
#include <sensor_msgs/msg/laser_scan.h>

//内部有这2个控制引脚，但是我不知道他们是有有生效。
//15  19脚其实我的电机没有
/**
 * @class LidarManager
 * @brief 高可用自主雷达协议解析引擎
 * 针对 YDLidar x3pro 系列设计的工业级解析器，具备自动同步修复与噪声抑制功能。
 */
class LidarManager {
public:
    LidarManager();
    ~LidarManager();

    // 引擎初始化
    void begin(const char* frame_id = "laser_frame");
    // 异步数据流迭代
    void update();

    sensor_msgs__msg__LaserScan& getLaserScanMsg() { return scan_msg; }

    bool isScanReady() { return scan_ready; }
    void resetScanReady() { scan_ready = false; }
    
    // 健康审计指标
    uint32_t getCorruptedCount() { return corrupted_packets; }
    uint32_t getTotalProcessed() { return total_packets; }

private:
    enum State { 
        WAIT_SYNC1, WAIT_SYNC2, WAIT_TYPE, WAIT_COUNT, 
        WAIT_S_ANG, WAIT_E_ANG, WAIT_DATA, WAIT_CHECK 
    };
    
    State state = WAIT_SYNC1;
    
    uint8_t  packet_type;
    uint8_t  sample_count;
    uint16_t start_angle_raw;
    uint16_t end_angle_raw;
    uint16_t checksum_received;
    
    uint8_t  data_buffer[160]; 
    int      data_idx = 0;

    sensor_msgs__msg__LaserScan scan_msg;
    bool scan_ready = false;
    const uint16_t PUB_SCAN_SIZE = 360;
    
    uint32_t corrupted_packets = 0;
    uint32_t total_packets = 0;

    void processFullPacket();
    bool validateDataIntegrity();
    void initScanMsg(const char* frame_id);
    
    const uint8_t LIDAR_PWM_PIN = 17;
    // const uint8_t LIDAR_EN_PIN = 19;
};

#endif
