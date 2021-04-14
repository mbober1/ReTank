#include <motor.hpp>
#include <packet.hpp>

struct RobotConfig
{
    //motor gpios
    gpio_num_t in1;
    gpio_num_t in2;
    gpio_num_t in3;
    gpio_num_t in4;
    
    //motor pwms
    gpio_num_t pwm1;
    gpio_num_t pwm2;
    ledc_channel_t pwmChannel;

    //encoder gpios
    gpio_num_t enc1a;
    gpio_num_t enc1b;
    gpio_num_t enc2a;
    gpio_num_t enc2b;

    pcnt_unit_t pcntUnit1;
    pcnt_unit_t pcntUnit2;

};


class Robot
{
private:
public:
    motor* engine;
    Robot(const RobotConfig &config);
    void drive(EnginePacket &packet);
};

Robot::Robot(const RobotConfig &config)
{
    engine = new motor[2] { 
        motor(config.in1, config.in2, config.enc1a, config.enc1b, config.pwm1, config.pwmChannel, config.pcntUnit1), 
        motor(config.in3, config.in4, config.enc2a, config.enc2b, config.pwm2, config.pwmChannel, config.pcntUnit2) 
    };
}

inline void Robot::drive(EnginePacket &packet) {
    engine[0].drive(packet.left);
    engine[1].drive(packet.right);
}