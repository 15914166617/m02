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


//声明ros2实体
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//订阅者
rcl_subscription_t cmd_sub;
geometry_msgs__msg__Twist msg_cmd;


//设置loop_ping的超时时间
float ping_prev_pub_time_us = 0;
#define UROS_PING_PUB_PERIOD_US 3000000 
//设置硬件与运动学参数(在订阅的cmdvel和里程计融合有用到)
const float WHEEL_SEPARATION = 0.174; // 轮距 (m)
const float WHEEL_RADIUS = 0.0325;    // 轮半径 (m)
DataConverter converter(WHEEL_SEPARATION, WHEEL_RADIUS);
//创建电机控制实体
Motors motors;


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
//订阅回调函数，接受下发的线速度和角速度，变成左右轮子的pid目标速度
void cmd_vel_callback(const void * msin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msin;
    float v = msg->linear.x;
    float w = msg->angular.z;
    float left = v - (w * WHEEL_SEPARATION / 2.0f);
    float right = v + (w * WHEEL_SEPARATION / 2.0f);
    motors.setTargetSpeeds(left, right);
}

//保活函数
void loop_ping() {
    static int retry_count = 0; 
    const int MAX_RETRIES = 15;  // 设置为 15 次（约 45 秒），给上位机充足的启动时间
    
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


    //设置并且连接WIFI
    WiFi.setSleep(false);//wifi设置为不休眠不降低频率
    //设置wifi的帐号密码地址端口，超时时间                                    （needadd）
    if (!TransportManager::init(15000)) {
        // 如果连接失败的预警处理
        Serial.println("System halt: Transport failed.");
        while(1) delay(1000);
    }
    Serial.println("\n[SYSTEM] Wifi_Init_OK...");


    //初始化ros2功能
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "m02_node", "", &support));

    // 初始化速度指令订阅者
    RCCHECK(rclc_subscription_init_default(&cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

    //初始化时间管理器
    URosTimeManager::getInstance().begin(5000);
    Serial.println("System Initialized, Time is begin.");

    //初始化电机控制器（配置引脚、创建对象、启动PID）
    motors.begin();
    Serial.println("Robot Motors Initialized...");

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
double dt = motors.update();
    
//自有延迟
delay(1);
}
