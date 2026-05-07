#ifndef DATA_CONVERTER_H
#define DATA_CONVERTER_H

#include <micro_ros_platformio.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <geometry_msgs/msg/quaternion.h>
#include <math.h>

/**

@class DataConverter

@brief 数据转换与融合类：负责差速底盘运动学计算及消息封装
*/
class DataConverter {
public:
DataConverter(float wheel_separation, float wheel_radius);

/**

@brief 更新航位推算 (Dead Reckoning)

@param left_vel 左轮速度 (m/s)

@param right_vel 右轮速度 (m/s)

@param dt 时间步长 (s)
*/
void updateOdometry(float left_vel, float right_vel, float dt);

// 获取消息引用
nav_msgs__msg__Odometry& getOdomMsg() { return odom_msg_; }
// sensor_msgs__msg__LaserScan& getScanMsg() { return scan_msg_; }

private:
float wheel_separation_;
float wheel_radius_;

// 位姿状态 (x, y, yaw)
double x_ = 0, y_ = 0, theta_ = 0;

nav_msgs__msg__Odometry odom_msg_;
// sensor_msgs__msg__LaserScan scan_msg_;

void initMessages();


};

#endif