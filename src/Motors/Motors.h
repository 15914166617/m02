#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>

class Motors {
public:
Motors();
void begin();

/**
 * @brief 更新电机状态：读取编码器、计算 PID 并输出 PWMssss
 * @return 返回本次更新的时间跨度 dt (s)
 */
double update();

void setTargetSpeeds(double leftMS, double rightMS);

double getLeftSpeed() const { return currentSpeedL; }
double getRightSpeed() const { return currentSpeedR; }


private:
void driveMotor(int pin1, int pin2, double pwm);

Encoder *encLeft, *encRight;
PID *pidLeft, *pidRight;

// 物理参数：需要根据实际轮子和减速比校准
// const double TICKS_PER_METER = 12066.6; 
// 减速比56，一圈霍尔11，然后好像有其他问题,比理论之小0.54倍数
const double TICKS_PER_METER = 6542.0; 

double targetSpeedL, currentSpeedL, outputL;
double targetSpeedR, currentSpeedR, outputR;
long lastPosL, lastPosR;
unsigned long lastUpdateTime;

// 引脚分配
// static const uint8_t L_ENC_A = 32, L_ENC_B = 35, L_IN1 = 27, L_IN2 = 26;
//回家后左轮不知道为什么方向反了，现在改控制输入
static const uint8_t L_ENC_A = 35, L_ENC_B = 32, L_IN1 = 26, L_IN2 = 27;

static const uint8_t R_ENC_A = 34, R_ENC_B = 39, R_IN1 = 25, R_IN2 = 33;


};

#endif