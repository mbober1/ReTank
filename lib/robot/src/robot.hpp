#include <motor.hpp>
#include <packet.hpp>

class robot
{
private:
public:
    motor* engine;

    robot(gpio_num_t in1, gpio_num_t in2, uint8_t pwmPin, uint8_t pwmChannel, gpio_num_t in3, gpio_num_t in4, uint8_t pwmPin2, uint8_t encoder1A, uint8_t encoder1B, uint8_t encoder2A, uint8_t encoder2B, pcnt_unit_t pcntUnit1, pcnt_unit_t pcntUnit2);
    void drive(EnginePacket &packet);
};

robot::robot(gpio_num_t in1, gpio_num_t in2, uint8_t pwmPin, uint8_t pwmChannel, gpio_num_t in3, gpio_num_t in4, uint8_t pwmPin2, uint8_t encoder1A, uint8_t encoder1B, uint8_t encoder2A, uint8_t encoder2B, pcnt_unit_t pcntUnit1, pcnt_unit_t pcntUnit2)
{
    engine = new motor[2] { 
        motor(in1, in2, encoder1A, encoder1B, pwmPin, pwmChannel, pcntUnit1), 
        motor(in3, in4, encoder2A, encoder2B, pwmPin, pwmChannel, pcntUnit2) 
    };
}

inline void robot::drive(EnginePacket &packet) {
    engine[0].drive(packet.left);
    // engine[1].drive(packet.right);
}