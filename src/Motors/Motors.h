#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>

// const double Motors::TICKS_PER_METER= 6542.0; 
// const uint8_t Motors::L_ENC_A= 35, Motors::L_ENC_B = 32, Motors::L_IN1 = 26, Motors::L_IN2 = 27; // 左轮引脚
// const uint8_t Motors::R_ENC_A = 34, Motors::R_ENC_B = 39, Motors::R_IN1 = 25, Motors::R_IN2 = 33; // 右轮引脚
// const double Motors::PID_KP=125,Motors::PID_KI=1500,Motors::PID_KD=0.5;

class Motors {
public:

    //减速比和引脚配置
    // 物理参数：需要根据实际轮子和减速比校准
    // const double TICKS_PER_METER = 12066.6; 
    // 减速比56，一圈霍尔11，然后好像有其他问题,比理论之小0.54倍数
    static const double TICKS_PER_METER; 
    // 引脚分配
    //回家后左轮不知道为什么方向反了，现在改控制输入
    static const uint8_t L_ENC_A, L_ENC_B, L_IN1, L_IN2;
    static const uint8_t R_ENC_A, R_ENC_B, R_IN1, R_IN2;
    // static const uint8_t L_ENC_A = 32, L_ENC_B = 35, L_IN1 = 27, L_IN2 = 26;

    static const double PID_KP,PID_KI,PID_KD;

    // Motors();结合下面构造函数放私有，可以让类只有一个实体
    static Motors& getInstance() {
        static Motors instance;
        return instance;
    }

    //配置引脚，驱动范围，霍尔，pid参数
    void begin();

    //更新电机状态：读取编码器，计算pid,输出pwm,返回本次更新的时间跨度dt(s)
    double update();

    //设置目标速度,当前项目中在订阅回调处理函数中，换算完线速度和角速度为左右轮子速度后，设置进pid目标中
    void setTargetSpeeds(double leftMS, double rightMS);

    //获取电机速度，当前项目中，loop循环计算里程计时调用
    double getLeftSpeed() const { return currentSpeedL; }
    double getRightSpeed() const { return currentSpeedR; }


private:
    // Motors();结合上面的引用调用，可以让类只有一个实体
    Motors();

    //执行电机速度
    void driveMotor(int pin1, int pin2, double pwm);

    Encoder *encLeft, *encRight;
    PID *pidLeft, *pidRight;

    double targetSpeedL, currentSpeedL, outputL;
    double targetSpeedR, currentSpeedR, outputR;
    long lastPosL, lastPosR;
    unsigned long lastUpdateTime;

};

#endif