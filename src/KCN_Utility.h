#ifndef __KCN_UTILITY_H
#define __KCN_UTILITY_H

#define POSITIVE 0x1
#define NEGATIVE 0x0

#define ANALOG true
#define DIGITAL false

#include "Arduino.h"
#include "Wire.h"

uint8_t i2cScanner();
uint8_t i2cReceiveUint8(uint8_t address);
uint8_t i2cReceiveUint16(uint8_t address);
uint8_t i2cReceiveUint32(uint8_t address);
uint8_t i2cReceiveUint64(uint8_t address);

enum i2cReadBytes
{
    I2C_READ_1 = 1,
    I2C_READ_2 = 2,
    I2C_READ_4 = 4,
    I2C_READ_8 = 8
};

struct Button
{
    uint8_t pin;
    bool read;
    bool record;

    Button(uint8_t pin);
    Button(uint8_t pin, uint8_t mode);

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

struct i2cSensor
{
    uint8_t i2cADDR;
    uint8_t SDA_PIN;
    uint8_t SCL_PIN;
    i2cReadBytes dataBytes;

    i2cSensor(uint8_t SDA_PIN, uint8_t SCL_PIN, uint8_t i2cADDR, i2cReadBytes dataBytes);
    i2cSensor(uint8_t SDA_PIN, uint8_t SCL_PIN, uint8_t i2cADDR);
    i2cSensor(uint8_t SDA_PIN, uint8_t SCL_PIN);

    void doRead(uint8_t &val);
    void doRead(uint16_t &val);
    void doRead(uint32_t &val);
    void doRead(uint64_t &val);

    void autoRead(uint64_t &val);
};

#endif