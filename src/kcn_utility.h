#ifndef __KCN_UTILITY_H
#define __KCN_UTILITY_H

#define POSITIVE = 0x1
#define NEGATIVE = 0x0

#include "Arduino.h"

struct Button
{
    uint8_t pin;
    bool read = 1;
    bool record = 1;

    Button(uint8_t pin);

    void doRead();
    void doRecord();
    bool isOnClick();
};

struct StepperMotor
{
    uint8_t ENA_PIN;
    uint8_t DIR_PIN;
    uint8_t STEP_PIN;

    StepperMotor(uint8_t ENA_PIN, uint8_t DIR_PIN, uint8_t STEP_PIN);
    StepperMotor(uint8_t DIR_PIN, uint8_t STEP_PIN);

    void enable();
    void disable();
    void setDirection(uint8_t direction);
    void stepPulse(int delay_us);
};

#endif