#include "transport_manager.h"
#include <WiFi.h>

// 静态成员变量定义
// const char* TransportManager::SSID       = "zrc";
// const char* TransportManager::PASSWORD   = "z15914166617";
// const char* TransportManager::AGENT_IP   = "192.168.0.103";
// const uint16_t TransportManager::AGENT_PORT = 8888;

bool TransportManager::init() {
    Serial.begin(115200);
    Serial.println("\n[Transport] Starting initialization...");

    // 修复：将字符串 IP 转换为 ESP32 专用的 IPAddress 类型
    IPAddress agent_ip_obj;
    if (!agent_ip_obj.fromString(AGENT_IP)) {
        Serial.println("[Transport] Error: Invalid IP Address format!");
        return false;
    }

    // 配置 micro-ROS WiFi 传输层
    // 注意：这里传入解析后的 agent_ip_obj
    set_microros_wifi_transports(
        (char*)SSID, 
        (char*)PASSWORD, 
        agent_ip_obj, 
        AGENT_PORT
    );

    // 等待 WiFi 连接的逻辑检查
    uint32_t start_time = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start_time) < TIMEOUT_MS) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[Transport] WiFi Connected!");
        Serial.print("[Transport] IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.printf("[Transport] Target Agent: %s:%d\n", AGENT_IP, AGENT_PORT);
        return true;
    } else {
        Serial.println("\n[Transport] Connection Timeout!");
        return false;
    }
}

bool TransportManager::isWifiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

bool TransportManager::isAgentReachable() {
    return isWifiConnected();
}