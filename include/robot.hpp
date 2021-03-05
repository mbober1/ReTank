#include "motor.hpp"
#include "math.h"

class robot
{
private:
    motor* engine;
public:
    robot(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t pwmChannel, uint8_t in3, uint8_t in4, uint8_t pwmPin2, uint8_t pwmChannel2);
    void drive(int left, int right);
};

robot::robot(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t pwmChannel, uint8_t in3, uint8_t in4, uint8_t pwmPin2, uint8_t pwmChannel2)
{
    engine = new motor[2] { motor(in1, in2, pwmPin, pwmChannel), motor(in3, in4, pwmPin2, pwmChannel2) };
}

void robot::drive(int left, int right) {
    if(left>0) engine[0].direction(Direction::FORWARD);
    else if(left<0) engine[0].direction(Direction::BACKWARD);
    else engine[0].softStop();

    if(right>0) engine[1].direction(Direction::FORWARD);
    else if(right<0) engine[1].direction(Direction::BACKWARD);
    else engine[1].softStop();

    engine[0].power(abs(left));
    engine[1].power(abs(right));
}