#include "Motors.h"

Motors::Motors() : lastUpdateTime(0), lastPosL(0), lastPosR(0) {
    targetSpeedL = targetSpeedR = 0;
    currentSpeedL = currentSpeedR = 0;
    outputL = outputR = 0;
}

void Motors::begin() {
    pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);

    encLeft = new Encoder(L_ENC_A, L_ENC_B);
    encRight = new Encoder(R_ENC_A, R_ENC_B);

    // Kp, Ki, Kd 参数：200.0, 1000.0, 1.0 (根据经验设定，建议实际调节)
    // pidLeft = new PID(&currentSpeedL, &outputL, &targetSpeedL, 200.0, 1000.0, 1.0, DIRECT);
    // pidRight = new PID(&currentSpeedR, &outputR, &targetSpeedR, 200.0, 1000.0, 1.0, DIRECT);
    pidLeft = new PID(&currentSpeedL, &outputL, &targetSpeedL, 125, 1500, 0.5, DIRECT);
    pidRight = new PID(&currentSpeedR, &outputR, &targetSpeedR, 125, 1500, 0.5, DIRECT);

    pidLeft->SetMode(AUTOMATIC);
    pidLeft->SetOutputLimits(-255, 255);
    pidLeft->SetSampleTime(20); 

    pidRight->SetMode(AUTOMATIC);
    pidRight->SetOutputLimits(-255, 255);
    pidRight->SetSampleTime(20);
}

double Motors::update() {
    unsigned long now = millis();
    if (lastUpdateTime == 0) {
        lastUpdateTime = now;
        lastPosL = encLeft->read();
        lastPosR = encRight->read();
        return 0;
    }

    double dt = (now - lastUpdateTime) / 1000.0;
    if (dt < 0.01) return 0; // 避免计算过于频繁导致数值不稳定

    long currPosL = encLeft->read();
    long currPosR = encRight->read();

    // 速度计算：(脉冲增量 / 比例常数) / 时间 = 米/秒
    currentSpeedL = (double)(currPosL - lastPosL) / TICKS_PER_METER / dt;
    currentSpeedR = (double)(currPosR - lastPosR) / TICKS_PER_METER / dt;

    // PID 调节
    pidLeft->Compute();
    pidRight->Compute();

    // PWM 输出
    driveMotor(L_IN1, L_IN2, outputL);
    driveMotor(R_IN1, R_IN2, outputR);

    // Serial.println(outputL);
    // Serial.println(outputR);

    lastPosL = currPosL;
    lastPosR = currPosR;
    lastUpdateTime = now;

    return dt;
}

void Motors::setTargetSpeeds(double leftMS, double rightMS) {
    targetSpeedL = leftMS;
    targetSpeedR = rightMS;
}

void Motors::driveMotor(int pin1, int pin2, double pwm) {
    int minPWM = 30; // 这里的 30 是补偿量
    int speed = constrain(abs((int)pwm), 0, 255);

    if (abs(pwm) < 1) {
        analogWrite(pin1, 0);
        analogWrite(pin2, 0);
    } else if (pwm > 0) {
        analogWrite(pin1, speed+minPWM);
        analogWrite(pin2, 0);
    } else {
        analogWrite(pin1, 0);
        analogWrite(pin2, speed+minPWM);
    }
    // Serial.print("Final Speed: "); Serial.println(speed);
}