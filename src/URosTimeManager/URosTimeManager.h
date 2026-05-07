

#ifndef UROS_TIME_MANAGER_H
#define UROS_TIME_MANAGER_H

#include <rcl/rcl.h>
#include <builtin_interfaces/msg/time.h>
#include <rmw_microros/rmw_microros.h>
#include <time.h>
#include <sys/time.h>

class URosTimeManager {
public:
    static URosTimeManager& getInstance() {
        static URosTimeManager instance;
        return instance;
    }

    /**
     * @brief 初始化配置
     * @param period_ms 同步周期（毫秒）
     */
    void begin(uint32_t period_ms = 60000);

    /**
     * @brief 在主循环中调用的轮询函数（非阻塞）
     * 内部会自动判断时间并执行同步
     */
    void update();

    /**
     * @brief 立即执行一次同步
     */
    bool syncNow();

    /**
     * @brief 获取当前同步后的 ROS 2 时间戳
     */
    builtin_interfaces__msg__Time getRosTime();

    bool isSynced() const { return is_synced_; }

private:
    URosTimeManager() : is_synced_(false), sync_period_ms_(60000), last_sync_ms_(0) {}
    
    bool is_synced_;
    uint32_t sync_period_ms_;
    uint32_t last_sync_ms_;
};

#endif