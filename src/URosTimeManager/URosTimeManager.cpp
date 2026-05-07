// #include "URosTimeManager.h"
// #include <Arduino.h> // 仅用于日志打印，如果是纯 IDF 可更换

// void URosTimeManager::begin(uint32_t period_ms) {
//     sync_period_ms_ = period_ms;
    
//     // 创建后台同步任务
//     // 优先级设为较小值，避免干扰传感器采集
//     xTaskCreate(
//         syncTask,
//         "uros_time_sync",
//         4096,
//         this,
//         1,
//         NULL
//     );
// }

// bool URosTimeManager::syncNow() {
//     // 尝试与 Agent 同步 session 时间
//     rmw_ret_t ret = rmw_uros_sync_session(1000);
    
//     if (ret == RCL_RET_OK) {
//         int64_t ns = rmw_uros_epoch_nanos();
        
//         struct timeval tv;
//         tv.tv_sec = (time_t)(ns / 1000000000);
//         tv.tv_usec = (suseconds_t)((ns % 1000000000) / 1000);
        
//         // 更新内核时钟 (CLOCK_REALTIME)
//         if (settimeofday(&tv, NULL) == 0) {
//             is_synced_ = true;
//             return true;
//         }
//     }
//     return false;
// }

// builtin_interfaces__msg__Time URosTimeManager::getRosTime() {
//     struct timespec now;
//     // 使用 POSIX 标准函数获取内核时间，精度为纳秒
//     clock_gettime(CLOCK_REALTIME, &now);
    
//     builtin_interfaces__msg__Time ros_time;
//     ros_time.sec = (int32_t)now.tv_sec;
//     ros_time.nanosec = (uint32_t)now.tv_nsec;
//     return ros_time;
// }

// void URosTimeManager::syncTask(void* pvParameters) {
//     URosTimeManager* manager = (URosTimeManager*)pvParameters;
    
//     // 给系统留一点初始化时间
//     vTaskDelay(pdMS_TO_TICKS(5000));

//     while (true) {
//         // 只有在 Agent 连接正常时才同步
//         if (rmw_uros_ping_agent(100, 1) == RCL_RET_OK) {
//             if (manager->syncNow()) {
//                 // 同步成功后，可以根据需要打印日志
//                 // Serial.println("[TimeSync] Successfully synced with Agent");
//             }
//         }
        
//         // 进入周期性休眠
//         vTaskDelay(pdMS_TO_TICKS(manager->sync_period_ms_));
//     }
// }

#include "URosTimeManager.h"
#include <Arduino.h>

void URosTimeManager::begin(uint32_t period_ms) {
    sync_period_ms_ = period_ms;
    last_sync_ms_ = millis(); 
    is_synced_ = false;
    // 删除了 xTaskCreate，确保所有网络操作回归主线程
}

void URosTimeManager::update() {
    // 检查是否到了同步周期
    if (millis() - last_sync_ms_ >= sync_period_ms_) {
        last_sync_ms_ = millis();
        
        // 仅在网络连通时尝试同步，避免长时间阻塞
        if (rmw_uros_ping_agent(50, 1) == RCL_RET_OK) {
            syncNow();
        }
    }
}

bool URosTimeManager::syncNow() {
    // 这里的超时时间缩短，避免阻塞主循环太久
    rmw_ret_t ret = rmw_uros_sync_session(100);
    
    if (ret == RCL_RET_OK) {
        int64_t ns = rmw_uros_epoch_nanos();
        
        struct timeval tv;
        tv.tv_sec = (time_t)(ns / 1000000000);
        tv.tv_usec = (suseconds_t)((ns % 1000000000) / 1000);
        
        if (settimeofday(&tv, NULL) == 0) {
            is_synced_ = true;
            return true;
        }
    }
    return false;
}

builtin_interfaces__msg__Time URosTimeManager::getRosTime() {
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    
    // 防御性处理：如果时间戳秒数过小（未同步成功），则返回 0
    // 避免给上位机发送负值或 1970 年的时间戳
    builtin_interfaces__msg__Time ros_time;
    if (now.tv_sec < 1000000) {
        ros_time.sec = 0;
        ros_time.nanosec = 0;
    } else {
        ros_time.sec = (int32_t)now.tv_sec;
        ros_time.nanosec = (uint32_t)now.tv_nsec;
    }
    return ros_time;
}