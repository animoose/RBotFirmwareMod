// RBotFirmware
// Rob Dobson 2016

#pragma once

#include <Adafruit_MotorShield.h>
#include <Arduino.h>
#include "RobotConsts.h"

class StepperMotor
{
  private:
    // Motor type
    RobotConsts::MOTOR_TYPE _motorType;

    // Motor operates in reverse (i.e. direction low = forwards)
    bool _motorDirectionReversed;

    // Minimum width of stepping pulse
    int _minPulseWidthUs;

    // Pins for the motor
    int _pinStep;
    int _pinDirection;

    // Current step pin level
    bool _pinStepLevel;

    // Adafruit stepper object, for MOTOR_TYPE_SHIELD only.
    Adafruit_StepperMotor *_adafruit_stepper;

  public:
    // For MOTOR_TYPE_DRIVER two pins are used step & direction
    StepperMotor(RobotConsts::MOTOR_TYPE motorType, uint8_t pinStep, uint8_t pinDirection, bool directionReversed) :
        _adafruit_stepper(nullptr)
    {
        if (motorType == RobotConsts::MOTOR_TYPE_DRIVER)
        {
            if (pinStep != -1 && pinDirection != -1)
            {
                _motorType = motorType;
                _motorDirectionReversed = directionReversed;
                _minPulseWidthUs = 1;
                // Setup the pins
                pinMode(pinStep, OUTPUT);
                _pinStep = pinStep;
                pinMode(pinDirection, OUTPUT);
                _pinDirection = pinDirection;
                _pinStepLevel = 0;
            }
        }
        else
        {
            motorType = RobotConsts::MOTOR_TYPE_NONE;
        }
    }

    // Construct a StepperMotor using the Adafruit Motor Shield V2, for motor |motorNum| on |shield|.
    // NOTE: most of the methods in this class have not been implemented for this case, as the motor
    // control is done directly in MotionActuator.
    StepperMotor(Adafruit_MotorShield *shield, int motorNum) {
        _motorType = RobotConsts::MOTOR_TYPE_ADAFRUIT;
        // Set up with 200 steps per revolution. The value here does not matter as we will be accessing
        // the stepper through the onestep method, which does not depend on this value.
        _adafruit_stepper = shield->getStepper(200, motorNum);

        // Initialize the other values to keep the object well-defined.
        _motorDirectionReversed = false;
        _minPulseWidthUs = 1;
        _pinStep = -1;
        _pinDirection = -1;
        _pinStepLevel = 0;
    }

    ~StepperMotor()
    {
        deinit();
    }

    void deinit()
    {
        _motorType = RobotConsts::MOTOR_TYPE_NONE;
        // Free up pins
        if (_pinStep != -1)
            pinMode(_pinStep, INPUT);
        if (_pinDirection != -1)
            pinMode(_pinDirection, INPUT);
        if (_adafruit_stepper) {
            _adafruit_stepper->release();
            _adafruit_stepper = nullptr;
        }
    }

    // Set direction
    void stepDirn(bool dirn)
    {
        digitalWrite(_pinDirection, _motorDirectionReversed ? dirn : !dirn);
    }

    bool stepEnd()
    {
        if (!_pinStepLevel)
            return false;
        digitalWrite(_pinStep, false);
        _pinStepLevel = false;
        return true;
    }

    void stepStart()
    {
        digitalWrite(_pinStep, true);
        _pinStepLevel = true;
    }

    void stepSync(bool direction)
    {
        // Set direction
        digitalWrite(_pinDirection, _motorDirectionReversed ? direction : !direction);

        // Step
        digitalWrite(_pinStep, true);
        delayMicroseconds(_minPulseWidthUs);
        digitalWrite(_pinStep, false);
    }

    void getPins(int &stepPin, int &dirnPin, bool &dirnReverse)
    {
        stepPin = _pinStep;
        dirnPin = _pinDirection;
        dirnReverse = _motorDirectionReversed;
    }

    Adafruit_StepperMotor *getAdafruitStepper() {
        return _adafruit_stepper;
    }

    RobotConsts::MOTOR_TYPE getMotorType()
    {
        return _motorType;
    }
};
