#include "kcn_ultility.h"

/**
 * Constructor for the Button class.
 *
 * @param pin the pin number to which the button is connected
 */
Button::Button(uint8_t pin)
{
    this->pin = pin;
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
 * Check if the button is clicked.
 *
 * @return true if the button is clicked, false otherwise.
 */
bool Button ::isOnClick()
{
    return this->read && !this->record ? true : false;
}

StepperMotor::StepperMotor(uint8_t ENA_PIN, uint8_t DIR_PIN, uint8_t STEP_PIN)
{
    this->ENA_PIN = ENA_PIN;
    this->ENA_PIN = DIR_PIN;
    this->STEP_PIN = STEP_PIN;
}

StepperMotor::StepperMotor(uint8_t DIR_PIN, uint8_t STEP_PIN)
{
    StepperMotor(255, DIR_PIN, STEP_PIN);
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
