#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

//WiFi
#include "transport_manager/transport_manager.h"



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
}


void loop()
{


delay(1);
}
