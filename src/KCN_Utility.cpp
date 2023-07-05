#include "KCN_Utility.h"

/************* Basic I2C Functions *************/

/**
 * Scans the I2C bus for devices and returns the address of the first
 * device found.
 *
 * @return The address of the first device found, or 0 if no devices are
 *         found.
 */
uint8_t i2cScanner()
{
    for (uint8_t address = 1; address < 127; ++address)
    {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        if (error == 0)
        {
            return address;
        }
    }
    return 0;
}

/**
 * Receives a single 8-bit unsigned integer from an I2C device at the specified address.
 *
 * @param address the address of the I2C device
 *
 * @return the received 8-bit unsigned integer
 */
uint8_t i2cReceiveUint8(uint8_t address)
{
    uint8_t value = 0;

    Wire.requestFrom(address, 1);

    if (Wire.available() >= 1)
    {
        value = Wire.read();
    }

    return value;
}

/**
 * Receives a 16-bit unsigned integer via I2C communication.
 *
 * @param address the address of the device to communicate with
 *
 * @return the received 16-bit unsigned integer
 */
uint16_t i2cReceiveUint16(uint8_t address)
{
    uint16_t value = 0;

    Wire.requestFrom(address, 2);

    if (Wire.available() >= 2)
    {
        value = static_cast<uint16_t>(Wire.read()) << 8;
        value |= static_cast<uint16_t>(Wire.read());
    }

    return value;
}

/**
 * Receives a 32-bit unsigned integer from an I2C device at the specified address.
 *
 * @param address the address of the I2C device
 *
 * @return the received 32-bit unsigned integer
 */

uint32_t i2cReceiveUint32(uint8_t address)
{
    uint32_t value = 0;

    Wire.requestFrom(address, 4);

    if (Wire.available() >= 4)
    {
        value = static_cast<uint32_t>(Wire.read()) << 24;
        value |= static_cast<uint32_t>(Wire.read()) << 16;
        value |= static_cast<uint32_t>(Wire.read()) << 8;
        value |= static_cast<uint32_t>(Wire.read());
    }

    return value;
}

/**
 * Receives a 64-bit unsigned integer via I2C communication.
 *
 * @param address the I2C address to receive the value from
 *
 * @return the received 64-bit unsigned integer
 */
uint64_t i2cReceiveUint64(uint8_t address)
{
    uint64_t value = 0;

    Wire.requestFrom(address, 8);

    if (Wire.available() >= 8)
    {
        value = static_cast<uint64_t>(Wire.read()) << 56;
        value |= static_cast<uint64_t>(Wire.read()) << 48;
        value |= static_cast<uint64_t>(Wire.read()) << 40;
        value |= static_cast<uint64_t>(Wire.read()) << 32;
        value |= static_cast<uint64_t>(Wire.read()) << 24;
        value |= static_cast<uint64_t>(Wire.read()) << 16;
        value |= static_cast<uint64_t>(Wire.read()) << 8;
        value |= static_cast<uint64_t>(Wire.read());
    }

    return value;
}

/************* Button Structures *************/

/**
 * Constructor for the Button class.
 *
 * @param pin the pin number to which the button is connected
 */
Button::Button(uint8_t pin, uint8_t mode)
{
    this->pin = pin;
    pinMode(this->pin, mode);
}

Button::Button(uint8_t pin)
{
    this->pin = pin;
    pinMode(this->pin, INPUT_PULLUP);
}

/**
 * Reads the digital state of the button and stores the result in the
 * `read` member variable.
 */
void Button ::doRead()
{
    this->read = digitalRead(this->pin);
}

/**
 * Sets the value of the 'record' member variable to the value of the 'read'
 * member variable.
 */
void Button ::doRecord()
{
    this->record = this->read;
}

/**
 * Determines if the button is currently pressed down.
 *
 * @return true if the button is pressed down, false otherwise
 */
bool Button ::isKeyDown()
{
    return !(this->read || this->record);
}

/**
 * Check if the button is clicked.
 *
 * @return true if the button is clicked, false otherwise.
 */
bool Button ::isOnClick()
{
    return this->read && !this->record;
}

StepperMotor::StepperMotor(uint8_t ENA_PIN, uint8_t DIR_PIN, uint8_t STEP_PIN)
{
    this->ENA_PIN = ENA_PIN;
    this->ENA_PIN = DIR_PIN;
    this->STEP_PIN = STEP_PIN;
    if (this->ENA_PIN != 255)
    {
        pinMode(this->ENA_PIN, OUTPUT);
    }
    pinMode(this->DIR_PIN, OUTPUT);
    pinMode(this->STEP_PIN, OUTPUT);
}

/************* Stepper Motor Structures *************/
void StepperMotor::operator=(const StepperMotor &other)
{
    this->ENA_PIN = other.ENA_PIN;
    this->DIR_PIN = other.DIR_PIN;
    this->STEP_PIN = other.STEP_PIN;
}

StepperMotor::StepperMotor(uint8_t DIR_PIN, uint8_t STEP_PIN)
{
    this->ENA_PIN = 255;
    this->DIR_PIN = DIR_PIN;
    this->STEP_PIN = STEP_PIN;
}

/**
 * Enables the stepper motor.
 */
void StepperMotor::enable()
{
    if (this->ENA_PIN != 255)
    {
        digitalWrite(this->ENA_PIN, LOW);
    }
}

/**
 * Disables the stepper motor.
 */
void StepperMotor::disable()
{
    if (this->ENA_PIN != 255)
    {
        digitalWrite(this->ENA_PIN, HIGH);
    }
}

/**
 * Sets the direction of the stepper motor.
 *
 * @param direction the direction to set
 */
void StepperMotor ::setDirection(uint8_t direction)
{
    digitalWrite(this->DIR_PIN, direction);
}

/**
 * Executes a single step pulse for the stepper motor.
 *
 * @param delay_us the duration of the pulse in microseconds
 */
void StepperMotor::stepPulse(int delay_us)
{
    digitalWrite(this->STEP_PIN, HIGH);
    delayMicroseconds(delay_us);
    digitalWrite(this->STEP_PIN, LOW);
    delayMicroseconds(delay_us);
}

/************* Digital Sensor Structures *************/

DigitalSensor::DigitalSensor(uint8_t pin)
{
    this->pin = pin;
    pinMode(this->pin, INPUT);
}

/**
 * Reads the digital value from the sensor.
 *
 * @return the digital value read from the sensor
 */
bool DigitalSensor::doRead()
{
    return digitalRead(this->pin);
}

/************* Analog Sensor Structures *************/

AnalogSensor::AnalogSensor(uint8_t pin)
{
    this->pin = pin;
    pinMode(this->pin, INPUT);
}

/**
 * Reads the analog value from the sensor.
 *
 * @return the analog value read from the sensor
 */
uint16_t AnalogSensor::doRead()
{
    return analogRead(this->pin);
}

/************* Input Pin Structures *************/

InputPin::InputPin(uint8_t pin, bool InputMode)
{
    this->pin = pin;
    this->InputMode = InputMode;
    pinMode(this->pin, INPUT);
}

InputPin::InputPin(uint8_t pin)
{
    InputPin(pin, ANALOG);
}

/************* I2C Sensor Structures *************/

i2cSensor::i2cSensor(uint8_t SDA_PIN, uint8_t SCL_PIN, uint8_t i2cADDR, i2cReadBytes dataBytes)
{
    this->SDA_PIN = SDA_PIN;
    this->SCL_PIN = SCL_PIN;
    this->i2cADDR = i2cADDR;
    this->dataBytes = dataBytes;
}

i2cSensor::i2cSensor(uint8_t SDA_PIN, uint8_t SCL_PIN, uint8_t i2cADDRs)
{
    this->SDA_PIN = SDA_PIN;
    this->SCL_PIN = SCL_PIN;
    this->i2cADDR = i2cADDRs;
    this->dataBytes = I2C_READ_1;
}

i2cSensor::i2cSensor(uint8_t pin, uint8_t mode)
{
    i2cSensor(pin, mode, i2cScanner());
}

/**
 * Read a value from the I2C sensor.
 *
 * @param val a reference to store the read value
 */
void i2cSensor::doRead(uint8_t &val)
{
    val = i2cReceiveUint8(this->i2cADDR);
}

/**
 * Reads a uint16_t value from the I2C sensor.
 *
 * @param val the reference to store the read value
 *
 * @return void
 */
void i2cSensor::doRead(uint16_t &val)
{
    val = i2cReceiveUint16(this->i2cADDR);
}

/**
 * Reads a 32-bit value from an I2C sensor.
 *
 * @param val a reference to a uint32_t variable where the read value will be stored
 *
 * @return void
 */
void i2cSensor::doRead(uint32_t &val)
{
    val = i2cReceiveUint32(this->i2cADDR);
}

/**
 * Reads a 64-bit unsigned integer value from an I2C sensor.
 *
 * @param val a reference to the variable where the value will be stored
 *
 * @return void
 */
void i2cSensor::doRead(uint64_t &val)
{
    val = i2cReceiveUint64(this->i2cADDR);
}

/**
 * Reads data from an I2C sensor based on the number of data bytes.
 *
 * @param val a reference to a uint64_t variable to store the read value
 *
 * @return void
 */
void i2cSensor::autoRead(uint64_t &val)
{
    switch (this->dataBytes)
    {
    case I2C_READ_1:
        val = i2cReceiveUint8(this->i2cADDR);
        break;
    case I2C_READ_2:
        val = i2cReceiveUint16(this->i2cADDR);
        break;
    case I2C_READ_4:
        val = i2cReceiveUint32(this->i2cADDR);
        break;
    case I2C_READ_8:
        val = i2cReceiveUint64(this->i2cADDR);
        break;
    }
}

MecanumGroup::MecanumGroup(StepperMotor a, StepperMotor b, StepperMotor c, StepperMotor d)
    : motor{a, b, c, d} {}

/**
 * Drives the mecanum group in the specified direction.
 *
 * @param direction the direction in which to drive the mecanum group
 * @param delay_us the delay between each step pulse in microseconds
 */
void MecanumGroup::drive(SystemDirection direction, uint8_t delay_us)
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (direction & (1 << i))
        {

            this->motor[i].setDirection((direction & (1 << (i + 4))) ? POSITIVE : NEGATIVE);
            this->motor[i].stepPulse(delay_us);
        }
    }
}
