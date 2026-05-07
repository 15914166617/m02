#ifndef TRANSPORT_MANAGER_H
#define TRANSPORT_MANAGER_H

#include <Arduino.h>
#include <micro_ros_platformio.h>
//链接网络和agent
/**
 * @brief 传输层管理器类
 * 负责管理 WiFi 连接以及 micro-ROS Agent 的通信链路
 */
class TransportManager {
public:
    // 网络配置
    static const char* SSID;
    static const char* PASSWORD;
    static const char* AGENT_IP;
    static const uint16_t AGENT_PORT;

    /**
     * @brief 初始化网络传输层并连接到 Agent
     * @param timeout_ms 连接超时时间（毫秒）
     * @return true 初始化成功，false 初始化失败
     */
    static bool init(uint32_t timeout_ms = 10000);

    /**
     * @brief 检查当前 WiFi 是否连接
     * @return true 已连接
     */
    static bool isWifiConnected();

    /**
     * @brief 检查与 Agent 的链路状态 (ping 测试)
     * @return true 链路畅通
     */
    static bool isAgentReachable();
};

#endif