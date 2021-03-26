#include "driver/adc.h"

class myADC {
    const uint16_t R1 = 8112;
    const uint16_t R2 = 906;
    const float conversion_factor = (this->R1 + this->R2) * 1.317 / ((1 << 12) * this->R2);

public:
    myADC();
    float getVoltage();
};