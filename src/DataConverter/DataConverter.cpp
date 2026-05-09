#include "DataConverter.h"
#include <string.h>

DataConverter::DataConverter(float wheel_separation, float wheel_radius)
    : wheel_separation_(wheel_separation), wheel_radius_(wheel_radius) {
    initMessages();
}

void DataConverter::initMessages() {
    // 初始化里程计消息帧 ID
    odom_msg_.header.frame_id.data = (char*)"odom";
    odom_msg_.header.frame_id.size = strlen(odom_msg_.header.frame_id.data);
    // odom_msg_.child_frame_id.data = (char*)"base_link";
    odom_msg_.child_frame_id.data = (char*)"base_footprint";
    odom_msg_.child_frame_id.size = strlen(odom_msg_.child_frame_id.data);

    // 初始化协方差矩阵（简单起见，设为较小的值，表示信任度高）
    for(int i = 0; i < 36; i++) {
        odom_msg_.pose.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
        odom_msg_.twist.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
    }

    // 雷达消息基础参数配置
    // scan_msg_.header.frame_id.data = (char*)"laser_frame";
    // scan_msg_.header.frame_id.size = strlen(scan_msg_.header.frame_id.data);
}

void DataConverter::updateOdometry(float left_vel, float right_vel, float dt) {
    if (dt <= 0) return;

    // 1. 计算运动学增量
    double v = (right_vel + left_vel) / 2.0;
    double w = (right_vel - left_vel) / wheel_separation_;
    
    double delta_theta = w * dt;
    
    // 2. 使用中值积分提高精度
    double avg_theta = theta_ + (delta_theta / 2.0);
    x_ += v * cos(avg_theta) * dt;
    y_ += v * sin(avg_theta) * dt;
    theta_ += delta_theta;

    // 3. 角度归一化
    theta_ = atan2(sin(theta_), cos(theta_));

    // 4. 填充消息（确保包含时间戳）
    // odom_msg_.header.stamp = current_time; 
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    
    // 完美的 Z 轴旋转四元数
    odom_msg_.pose.pose.orientation.z = sin(theta_ / 2.0);
    odom_msg_.pose.pose.orientation.w = cos(theta_ / 2.0);

    odom_msg_.twist.twist.linear.x = v;
    odom_msg_.twist.twist.angular.z = w;
}