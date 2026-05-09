#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

//WiFi
#include "transport_manager/transport_manager.h"
//时间
#include "URosTimeManager/URosTimeManager.h"
//cmdvel
#include <geometry_msgs/msg/twist.h>
//motor
#include "Motors/Motors.h"
//数据融合odom
#include "DataConverter/DataConverter.h"
//里程计
#include <nav_msgs/msg/odometry.h>
//雷达
#include "LidarManager/LidarManager.h"
#include <sensor_msgs/msg/laser_scan.h>


//声明ros2实体
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//订阅者
rcl_subscription_t cmd_sub;
geometry_msgs__msg__Twist msg_cmd;
//发布者odom
rcl_publisher_t odom_pub;
//发布者scan
unsigned long last_scan_pub_time = 0;
rcl_publisher_t scan_pub;
//设置loop_ping保活的超时时间
float ping_prev_pub_time_us = 0;
// 推荐：1秒 (1,000,000微秒) 
#define UROS_PING_PUB_PERIOD_US (2000 * 1000)//2s
const int MAX_RETRIES = 9;  // 设置为 8 次（约 2*8=16 秒）


//里程计pub发送的计时
unsigned long last_odom_pub_time = 0;
const unsigned long ODOM_PUB_PERIOD_MS = 40; // 40ms 对应 25Hz，33ms 对应 30Hz

//设置硬件与运动学参数(在订阅的cmdvel和里程计融合有用到)
const float WHEEL_SEPARATION = 0.174; // 轮距 (m)
const float WHEEL_RADIUS = 0.0325;    // 轮半径 (m)
//数据融合实体
DataConverter converter(WHEEL_SEPARATION, WHEEL_RADIUS);
//创建电机控制实体
// Motors motors;
const double Motors::TICKS_PER_METER= 6542.0; 
const double Motors::PID_KP=125,Motors::PID_KI=1500,Motors::PID_KD=0.5;
const uint8_t Motors::L_ENC_A= 35, Motors::L_ENC_B = 32, Motors::L_IN1 = 26, Motors::L_IN2 = 27; // 左轮引脚
const uint8_t Motors::R_ENC_A = 34, Motors::R_ENC_B = 39, Motors::R_IN1 = 25, Motors::R_IN2 = 33; // 右轮引脚
// 创建雷达管理实例
LidarManager lidar;

//wifi 帐号 密码 运行agent的机器的ip 端口8888 连接超时ms
const char* TransportManager::SSID       = "zrc";
const char* TransportManager::PASSWORD   = "z15914166617";
const char* TransportManager::AGENT_IP   = "192.168.0.103";
const uint16_t TransportManager::AGENT_PORT = 8888;
const uint32_t TransportManager::TIMEOUT_MS = 10000;

// --- 错误检查宏定义 ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// 错误处理函数：如果初始化失败，LED快闪并停止运行(needadd是否自定义这个引脚灯泡)
void error_loop() {
    while(1) {
        digitalWrite(2, !digitalRead(2)); // 假设板载LED在引脚2
        delay(100);
    }
}
//订阅回调函数，接受下发的线速度和角速度，变成左右轮子的pid目标速度 的
void cmd_vel_callback(const void * msin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msin;
    float v = msg->linear.x;
    float w = msg->angular.z;
    float left = v - (w * WHEEL_SEPARATION / 2.0f);
    float right = v + (w * WHEEL_SEPARATION / 2.0f);
    // motors.setTargetSpeeds(left, right);
    Motors::getInstance().setTargetSpeeds(left, right);
}

//保活函数
void loop_ping() {
    static int retry_count = 0; 
    // const int MAX_RETRIES = 15;  // 设置为 15 次（约 45 秒），给上位机充足的启动时间
    
    unsigned long time_now_us = esp_timer_get_time();
    
    if (time_now_us - ping_prev_pub_time_us >= UROS_PING_PUB_PERIOD_US) { 
        ping_prev_pub_time_us = time_now_us;
        
        if (rmw_uros_ping_agent(100, 1) == RCL_RET_OK) {
            if(retry_count > 0) Serial.println("[SYSTEM] Connection active.");
            retry_count = 0;
        } else {
            retry_count++;
            // 日志频率，每 3 次失败才打印一次串口输出
            if (retry_count % 3 == 0) {
                Serial.printf("[WARN] Agent not responding (%d/%d)\n", retry_count, MAX_RETRIES);
            }
            
            if (retry_count >= MAX_RETRIES) {
                Serial.println("[ERROR] Connection lost for too long. Rebooting...");
                delay(500);
                ESP.restart(); 
            }
        }
    }
}


void setup()
{
    //调试串口配置
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n[SYSTEM] Serial_01_Init_OK...");

    //雷达串口设置，物理链路加固：显式配置 GPIO 模式，排除浮空干扰，强制 RX 引脚为上拉状态，防止悬空产生随机噪声;雷达串口初始化：预留充足的稳定时间，设置缓存大小;串口大小设置需要先设置大小再启动begin
    pinMode(16, INPUT_PULLUP); 
    Serial2.setRxBufferSize(2048);
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    
    Serial.println("\n[SYSTEM] Lidar_Serial_02_Init_OK...");
    delay(1000); 
    //雷达解析引擎启动
    lidar.begin("base_scan");
    Serial.println("\n[SYSTEM] Lidar Initialized_OK.");

    //设置并且连接WIFI
    WiFi.setSleep(false);//wifi设置为不休眠不降低频率
    //设置wifi的帐号密码地址端口，超时时间                                    （needadd）
    if (!TransportManager::init()) {
        // 如果连接失败的预警处理
        Serial.println("System halt: Transport failed.");
        while(1) delay(1000);
    }
    Serial.println("\n[SYSTEM] Wifi_Init_OK...");

    Serial.println("[SYSTEM] ROS2_Init_Start...");
    //初始化ros2功能
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "m02_node", "", &support));

    // 初始化速度指令订阅者
    RCCHECK(rclc_subscription_init_default(&cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    Serial.println("[SYSTEM] ROS2_cmd_vel_sub_Init_OK...");

    // 初始化里程计发布者
    RCCHECK(rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
    Serial.println("[SYSTEM] ROS2_odom_pub_Init_OK...");

    // 初始化雷达数据发布者
    RCCHECK(rclc_publisher_init_default(&scan_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"));
    Serial.println("[SYSTEM] ROS2_scan_pub_Init_OK...");

    // 为订阅者配置执行器和回调函数：单线程异步处理订阅回调
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &msg_cmd, &cmd_vel_callback, ON_NEW_DATA));
    Serial.println("[SYSTEM] ROS2_executor_Init_OK...");

    //初始化时间管理器
    URosTimeManager::getInstance().begin(5000);
    Serial.println("[SYSTEM] System Initialized, Time is begin.");
    URosTimeManager::getInstance().syncNow();
    Serial.println("[SYSTEM] ROS2_Time_Sync_OK...");
    

    //初始化电机控制器（配置引脚、创建对象、启动PID）
    // motors.begin();
    // Motors::getInstance(); // This line was redundant as the next line calls begin() on the same instance.
    Motors::getInstance().begin();
    Serial.println("Robot Motors Initialized...");
    Serial.println("[SYSTEM] All Initialized OK.");
    Serial.println("\n[SYSTEM] Setup finished. Starting main loop...");
    Serial.println("------------------------------------------------");
}


void loop()
{

//时间管理器轮询 (间隔由step中begin函数设置)
URosTimeManager::getInstance().update();

//保活心跳
loop_ping();

//释放时间给订阅处理订阅回调
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

//更新电机反馈（高频运行，确保 PID 控制精度），返回的dt时间，用于在计算了有效增量时进入odom的计算
// double dt = motors.update();
double dt = Motors::getInstance().update();

//Odom:出现有效增量,更新里程计内部位姿计算（必须每次都算，保证物理位置不丢失）    
if (dt > 0) {
        // 更新里程计内部位姿计算（必须每次都算，保证物理位置不丢失）
        converter.updateOdometry(Motors::getInstance().getLeftSpeed(), Motors::getInstance().getRightSpeed(), (float)dt);

        //话题发布频率控制:Odom
        unsigned long now = millis();
        if (now - last_odom_pub_time >= ODOM_PUB_PERIOD_MS) {
            last_odom_pub_time = now;
            // 获取待发布的里程计消息
            nav_msgs__msg__Odometry &odom = converter.getOdomMsg();
            // 填充同步后的时间戳
            struct timespec ns;
            clock_gettime(CLOCK_REALTIME, &ns);
            // --- 优化点：时间同步保护逻辑 ---
            if (ns.tv_sec < 1000000) { 
                // 如果时间还没同步，使用 ESP32 启动以来的毫秒数模拟时间戳
                // 这样 odom 话题会有频率，上位机能看到 TF 正在跳动
                odom.header.stamp.sec = millis() / 1000;
                odom.header.stamp.nanosec = (millis() % 1000) * 1000000;
            } else {
                odom.header.stamp.sec = ns.tv_sec;
                odom.header.stamp.nanosec = ns.tv_nsec;
            }
            // 执行发布（受频率限制，减轻网络负担）
            RCSOFTCHECK(rcl_publish(&odom_pub, &odom, NULL));
            // 可选：仅在发布时打印，避免串口过载
            /*
            Serial.print("L="); Serial.print(motors.getLeftSpeed());
            Serial.print(" R="); Serial.print(motors.getRightSpeed());
            Serial.print(" dt="); Serial.println(dt);
            */
        }
    }

//优先处理雷达串口数据，防止缓冲区溢出
lidar.update();
    if (lidar.isScanReady()) 
    {
        // static unsigned long last_scan_pub_time = 0;
        //雷达数据发布频率控制
        if (millis() - last_scan_pub_time >= 150) { // 限制在约 6.6Hz
            last_scan_pub_time = millis();
            // 填充同步后的时间戳
            struct timespec tv;
            clock_gettime(CLOCK_REALTIME, &tv);
            sensor_msgs__msg__LaserScan  &scan_msg = lidar.getLaserScanMsg();
            // --- 优化点：时间同步保护逻辑 ---
            if (tv.tv_sec < 1000000) { 
                // 如果时间还没同步，使用 ESP32 启动以来的毫秒数模拟时间戳
                // 这样 odom 话题会有频率，上位机能看到 TF 正在跳动
                scan_msg.header.stamp.sec = millis() / 1000;
                scan_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
            } else {
                scan_msg.header.stamp.sec = tv.tv_sec;
                scan_msg.header.stamp.nanosec = tv.tv_nsec;
            }

            // 执行发布（受频率限制，减轻网络负担）
            RCSOFTCHECK(rcl_publish(&scan_pub, &scan_msg, NULL));

        }
        lidar.resetScanReady();
    }
//自有延迟
delay(1);
}
