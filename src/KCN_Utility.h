#ifndef __KCN_UTILITY_H
#define __KCN_UTILITY_H

#define POSITIVE 0x1
#define NEGATIVE 0x0

#define ANALOG true
#define DIGITAL false

#include "Arduino.h"

struct Button
{
    uint8_t pin;
    bool read = 1;
    bool record = 1;

    Button(uint8_t pin);

    void doRead();
    void doRecord();
    bool isKeyDown();
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

struct DigitalSensor
{
    uint8_t pin;

    DigitalSensor(uint8_t pin);

    bool doRead();
};

struct AnalogSensor
{
    uint8_t pin;

    AnalogSensor(uint8_t pin);

    uint16_t doRead();
};

struct InputPin
{
    uint8_t pin;
    bool InputMode;

    InputPin(uint8_t pin, bool InputMode);
    InputPin(uint8_t pin);
};

struct MultiportSensor
{
    InputPin *pins;
    uint8_t count;
    uint16_t *readArray;

    MultiportSensor(InputPin *pins, uint8_t count, uint16_t *readArray);

    void doRead();
};

#endif