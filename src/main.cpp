#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

//WiFi
#include "transport_manager/transport_manager.h"
#include "URosTimeManager/URosTimeManager.h"


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


//声明ros2实体
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

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

    //初始化时间管理器
    URosTimeManager::getInstance().begin(5000);
    Serial.println("System Initialized, Time is begin.");



}


void loop()
{

//时间管理器轮询 (间隔由step中begin函数设置)
URosTimeManager::getInstance().update();

delay(1);
}
