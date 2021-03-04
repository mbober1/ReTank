#pragma once

enum class Direction {
    FORWARD,
    BACKWARD
};


class motor
{
private:
    uint8_t in1, in2, pwmPin, pwmChannel;
public:
    motor(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t pwmChannel);
    void direction(Direction dir);
    void power(uint8_t pow);
    void fastStop();
    void softStop();
};


motor::motor(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t pwmChannel) : in1(in1), in2(in2), pwmPin(pwmPin), pwmChannel(pwmChannel) {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    ledcSetup(pwmChannel, 2000, 8);
    ledcAttachPin(pwmPin, pwmChannel);
    softStop();
}


void motor::direction(Direction dir) {
    digitalWrite(in1, static_cast<uint8_t>(dir));
    digitalWrite(in2, !static_cast<uint8_t>(dir));
}

void motor::fastStop() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
}

void motor::softStop() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

void motor::power(uint8_t pow) {
    ledcWrite(pwmChannel, pow);
}