#include <motor.hpp>

    int16_t input1, input2;
    double e1, e2;

class robot
{
private:
public:
    motor* engine;

    robot(gpio_num_t in1, gpio_num_t in2, uint8_t pwmPin, uint8_t pwmChannel, gpio_num_t in3, gpio_num_t in4, uint8_t pwmPin2, uint8_t encoder1A, uint8_t encoder1B, uint8_t encoder2A, uint8_t encoder2B, pcnt_unit_t pcntUnit1, pcnt_unit_t pcntUnit2, pcnt_unit_t pcntUnit3, pcnt_unit_t pcntUnit4);
    void drive(int left, int right);
    void autos();
    void setPoint(uint16_t s1, uint16_t s2);
};

robot::robot(gpio_num_t in1, gpio_num_t in2, uint8_t pwmPin, uint8_t pwmChannel, gpio_num_t in3, gpio_num_t in4, uint8_t pwmPin2, uint8_t encoder1A, uint8_t encoder1B, uint8_t encoder2A, uint8_t encoder2B, pcnt_unit_t pcntUnit1, pcnt_unit_t pcntUnit2, pcnt_unit_t pcntUnit3, pcnt_unit_t pcntUnit4)
{
    engine = new motor[2] { 
        motor(in1, in2, encoder1A, encoder1B, pwmPin, pwmChannel, pcntUnit1, pcntUnit2), 
        motor(in3, in4, encoder2A, encoder2B, pwmPin, pwmChannel, pcntUnit3, pcntUnit4) 
    };
}

void robot::drive(int left, int right) {
    if(left>0) this->engine[0].direction(Direction::FORWARD);
    else if(left<0) this->engine[0].direction(Direction::BACKWARD);
    else this->engine[0].softStop();

    if(right>0) this->engine[1].direction(Direction::FORWARD);
    else if(right<0) this->engine[1].direction(Direction::BACKWARD);
    else this->engine[1].softStop();

    this->engine[0].power(abs(left));
    this->engine[1].power(abs(right));
}

void robot::setPoint(uint16_t s1, uint16_t s2) {
    engine[0].setpoint = s1;
    engine[1].setpoint = s2;
}


void robot::autos() {

    pcnt_get_counter_value(this->engine[0].encoder, &input1);
    // pcnt_get_counter_value(this->engine[0].encoder2, &input2);
    e1 = this->engine[0].compute();
    e2 = this->engine[1].compute();

    this->drive(((int)e1), ((int)e2));
}