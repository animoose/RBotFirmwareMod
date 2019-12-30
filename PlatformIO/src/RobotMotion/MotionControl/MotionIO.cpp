// RBotFirmware
// Rob Dobson 2016-18

#include "MotionIO.h"
#include <ESP32Servo.h>
#include "AxisValues.h"
#include "StepperMotor.h"
#include "EndStop.h"
#include "Utils.h"

static const char* MODULE_PREFIX = "MotionIO: ";

MotionIO::MotionIO()
{
    // Clear axis specific values
    for (int i = 0; i < RobotConsts::MAX_AXES; i++)
    {
        _stepperMotors[i] = NULL;
        _servoMotors[i] = NULL;
        for (int j = 0; j < RobotConsts::MAX_ENDSTOPS_PER_AXIS; j++)
            _endStops[i][j] = NULL;
    }
    // Stepper management
    _stepEnablePin = -1;
    _stepEnLev = true;
    _stepDisableSecs = 60.0;
    _motorEnLastMillis = 0;
    _motorEnLastUnixTime = 0;
    _adafruit_shield = nullptr;
}

MotionIO::~MotionIO()
{
    deinit();
}

void MotionIO::deinit()
{
    // disable
    if (_stepEnablePin != -1)
        pinMode(_stepEnablePin, INPUT);

    // remove motors and end stops
    for (int i = 0; i < RobotConsts::MAX_AXES; i++)
    {
        delete _stepperMotors[i];
        _stepperMotors[i] = NULL;
        delete _adafruit_shield;
        _adafruit_shield = nullptr;
        if (_servoMotors[i])
            _servoMotors[i]->detach();
        delete _servoMotors[i];
        _servoMotors[i] = NULL;
        for (int j = 0; j < RobotConsts::MAX_ENDSTOPS_PER_AXIS; j++)
        {
            delete _endStops[i][j];
            _endStops[i][j] = NULL;
        }
    }
}

bool MotionIO::configureAxis(const char *axisJSON, int axisIdx)
{
    if (axisIdx < 0 || axisIdx >= RobotConsts::MAX_AXES)
        return false;

    // Check the kind of motor to use
    bool isValid = false;
    String stepPinName = RdJson::getString("stepPin", "-1", axisJSON, isValid);
    if (isValid)
    {
        // Create the stepper motor for the axis
        String dirnPinName = RdJson::getString("dirnPin", "-1", axisJSON);
        int stepPin = ConfigPinMap::getPinFromName(stepPinName.c_str());
        int dirnPin = ConfigPinMap::getPinFromName(dirnPinName.c_str());
        bool directionReversed = (RdJson::getLong("dirnRev", 0, axisJSON) != 0);
        Log.notice("%sAxis%d (step pin %d, dirn pin %d)\n", MODULE_PREFIX, axisIdx, stepPin, dirnPin);

        String adafruitMotorNum = RdJson::getString("adafruitMotor", "", axisJSON);
        if (adafruitMotorNum.length() != 0)
        {
            // If we wanted more than two motors, we would need to allocate multiple shields here,
            // perhaps using the motor number to pick the address.
            // Note that the next couple of lines are not thread safe. I don't think this matters.
            if (_adafruit_shield == nullptr) {
                _adafruit_shield = new Adafruit_MotorShield(0x60);
                _adafruit_shield->begin(6400);
                Log.notice("%sAdafruit shield allocated\n", MODULE_PREFIX);
            }
            int motorNum = (int)strtol(adafruitMotorNum.c_str(), nullptr, 10);
            _stepperMotors[axisIdx] = new StepperMotor(_adafruit_shield, motorNum);
            Log.notice("%sAdafruit motor %d for axis %d\n", MODULE_PREFIX, motorNum, axisIdx);
        } else if (stepPin != -1 && dirnPin != -1)
        {
            _stepperMotors[axisIdx] = new StepperMotor(RobotConsts::MOTOR_TYPE_DRIVER, stepPin, dirnPin, directionReversed);
        }
    }
    else
    {
        // Create a servo motor for the axis
        String servoPinName = RdJson::getString("servoPin", "-1", axisJSON);
        long servoPin = ConfigPinMap::getPinFromName(servoPinName.c_str());
        Log.notice("%sAxis%d (servo pin %d)\n", MODULE_PREFIX, axisIdx, servoPin);
        if ((servoPin != -1))
        {
            _servoMotors[axisIdx] = new Servo();
            if (_servoMotors[axisIdx])
                _servoMotors[axisIdx]->attach(servoPin);
        }
    }

    // End stops
    for (int endStopIdx = 0; endStopIdx < RobotConsts::MAX_ENDSTOPS_PER_AXIS; endStopIdx++)
    {
        // Get the config for endstop if present
        String endStopIdStr = "endStop" + String(endStopIdx);
        String endStopJSON = RdJson::getString(endStopIdStr.c_str(), "{}", axisJSON);
        if (endStopJSON.length() == 0 || endStopJSON.equals("{}"))
            continue;

        // Create endStop from JSON
        _endStops[axisIdx][endStopIdx] = new EndStop(axisIdx, endStopIdx, endStopJSON.c_str());
    }

    return true;
}

bool MotionIO::configureMotors(const char *robotConfigJSON)
{
    // Get motor enable info
    String stepEnablePinName = RdJson::getString("stepEnablePin", "-1", robotConfigJSON);
    _stepEnLev = RdJson::getLong("stepEnLev", 1, robotConfigJSON);
    _stepEnablePin = ConfigPinMap::getPinFromName(stepEnablePinName.c_str());
    _stepDisableSecs = float(RdJson::getDouble("stepDisableSecs", stepDisableSecs_default, robotConfigJSON));
    Log.notice("%s(pin %d, actLvl %d, disableAfter %Fs)\n", MODULE_PREFIX, _stepEnablePin, _stepEnLev, _stepDisableSecs);

    if (_stepEnablePin != -1)
    {
        // Enable pin - initially disable
        pinMode(_stepEnablePin, OUTPUT);
        digitalWrite(_stepEnablePin, !_stepEnLev);
    }
    return true;
}

// Set step direction
void MotionIO::stepDirn(int axisIdx, bool dirn)
{
#ifdef BOUNDS_CHECK_ISR_FUNCTIONS
    _ASSERT(axisIdx >= 0 && axisIdx < RobotConsts::MAX_AXES);
#endif
    // Start dirn
    if (_stepperMotors[axisIdx])
        _stepperMotors[axisIdx]->stepDirn(dirn);
}

// Start a step
void MotionIO::stepStart(int axisIdx)
{
#ifdef BOUNDS_CHECK_ISR_FUNCTIONS
    _ASSERT(axisIdx >= 0 && axisIdx < RobotConsts::MAX_AXES);
#endif
    // Start step
    if (_stepperMotors[axisIdx])
        return _stepperMotors[axisIdx]->stepStart();
}

// Check if a step is in progress on any motor, if all such and return true, else false
bool MotionIO::stepEnd()
{
    // Check if step in progress
    bool aStepEnded = false;
    for (int axisIdx = 0; axisIdx < RobotConsts::MAX_AXES; axisIdx++)
    {
        if (_stepperMotors[axisIdx])
            aStepEnded = aStepEnded || _stepperMotors[axisIdx]->stepEnd();
    }
    return aStepEnded;
}

void MotionIO::stepSynch(int axisIdx, bool direction)
{
    if (axisIdx < 0 || axisIdx >= RobotConsts::MAX_AXES)
        return;

    enableMotors(true, false);
    if (_stepperMotors[axisIdx])
    {
        _stepperMotors[axisIdx]->stepSync(direction);
    }
    //_axisParams[axisIdx]._stepsFromHome += (direction ? 1 : -1);
    //_axisParams[axisIdx]._lastStepMicros = micros();
}

void MotionIO::jump(int axisIdx, long targetPosition)
{
    if (axisIdx < 0 || axisIdx >= RobotConsts::MAX_AXES)
        return;

    if (_servoMotors[axisIdx])
        _servoMotors[axisIdx]->writeMicroseconds(targetPosition);
}

// Endstops
bool MotionIO::isEndStopValid(int axisIdx, int endStopIdx)
{
    if (axisIdx < 0 || axisIdx >= RobotConsts::MAX_AXES)
        return false;
    if (endStopIdx < 0 || endStopIdx >= RobotConsts::MAX_ENDSTOPS_PER_AXIS)
        return false;
    return true;
}

bool MotionIO::isAtEndStop(int axisIdx, int endStopIdx)
{
    // For safety return true in these cases
    if (axisIdx < 0 || axisIdx >= RobotConsts::MAX_AXES)
        return true;
    if (endStopIdx < 0 || endStopIdx >= RobotConsts::MAX_ENDSTOPS_PER_AXIS)
        return true;

    // Test endstop
    if (_endStops[axisIdx][endStopIdx])
        return _endStops[axisIdx][endStopIdx]->isAtEndStop();

    // All other cases return true (as this might be safer)
    return true;
}

void MotionIO::getEndStopVals(AxisMinMaxBools& axisEndStopVals)
{
    for (int axisIdx = 0; axisIdx < RobotConsts::MAX_AXES; axisIdx++)
    {
        for (int endStopIdx = 0; endStopIdx < RobotConsts::MAX_ENDSTOPS_PER_AXIS; endStopIdx++)
        {
            AxisMinMaxBools::AxisMinMaxEnum endStopEnum = AxisMinMaxBools::END_STOP_NONE;
            if (_endStops[axisIdx][endStopIdx])
            {
                if (_endStops[axisIdx][endStopIdx]->isAtEndStop())
                    endStopEnum = AxisMinMaxBools::END_STOP_HIT;
                else
                    endStopEnum = AxisMinMaxBools::END_STOP_NOT_HIT;
            }
            axisEndStopVals.set(axisIdx, endStopIdx, endStopEnum);
        }
    }
}

void MotionIO::enableMotors(bool en, bool timeout)
{
    // Log.trace("Enable %d, disable level %d, disable after time %F\n",
    //							en, !_stepEnLev, _stepDisableSecs);
    if (en)
    {
        if (_stepEnablePin != -1)
        {
            if (!_motorsAreEnabled)
                Log.notice("%smotors enabled, disable after idle %Fs\n", MODULE_PREFIX, _stepDisableSecs);
            digitalWrite(_stepEnablePin, _stepEnLev);
        }
        _motorsAreEnabled = true;
        _motorEnLastMillis = millis();
        time(&_motorEnLastUnixTime);
    }
    else
    {
        if (_stepEnablePin != -1)
        {
            if (_motorsAreEnabled)
                Log.notice("%smotors disabled by %s\n", MODULE_PREFIX, timeout ? "timeout" : "command");
            digitalWrite(_stepEnablePin, !_stepEnLev);
        }
        _motorsAreEnabled = false;
    }
}

unsigned long MotionIO::getLastActiveUnixTime()
{
    return _motorEnLastUnixTime;
}

void MotionIO::motionIsActive()
{
    enableMotors(true, false);
}

void MotionIO::service()
{
    // Check for motor enable timeout
    if (_motorsAreEnabled && Utils::isTimeout(millis(), _motorEnLastMillis,
                                                (unsigned long)(_stepDisableSecs * 1000)))
        enableMotors(false, true);
}

void MotionIO::getRawMotionHwInfo(RobotConsts::RawMotionHwInfo_t &raw)
{
    // Fill in the info
    for (int axisIdx = 0; axisIdx < RobotConsts::MAX_AXES; axisIdx++)
    {
        // Initialise
        raw._axis[axisIdx]._pinStepCurLevel = 0;
        raw._axis[axisIdx]._motorType = RobotConsts::MOTOR_TYPE_NONE;
        raw._axis[axisIdx]._pinStep = -1;
        raw._axis[axisIdx]._pinDirection = -1;
        raw._axis[axisIdx]._pinDirectionReversed = 0;
        raw._axis[axisIdx]._pinEndStopMin = -1;
        raw._axis[axisIdx]._pinEndStopMinactLvl = 0;
        raw._axis[axisIdx]._pinEndStopMax = -1;
        raw._axis[axisIdx]._pinEndStopMaxactLvl = 0;
        raw._axis[axisIdx]._adafruit_stepper = nullptr;

        // Extract info about stepper motor if any
        if (_stepperMotors[axisIdx])
        {
            raw._axis[axisIdx]._motorType = _stepperMotors[axisIdx]->getMotorType();
            if (raw._axis[axisIdx]._motorType == RobotConsts::MOTOR_TYPE_ADAFRUIT) {
                raw._axis[axisIdx]._adafruit_stepper =
                    _stepperMotors[axisIdx]->getAdafruitStepper();
            } else {}
                _stepperMotors[axisIdx]->getPins(raw._axis[axisIdx]._pinStep,
                                                    raw._axis[axisIdx]._pinDirection,
                                                    raw._axis[axisIdx]._pinDirectionReversed);
        }
        // Min endstop
        if (_endStops[axisIdx][0])
        {
            _endStops[axisIdx][0]->getPins(raw._axis[axisIdx]._pinEndStopMin,
                                            raw._axis[axisIdx]._pinEndStopMinactLvl);
        }
        // Max endstop
        if (_endStops[axisIdx][1])
        {
            _endStops[axisIdx][1]->getPins(raw._axis[axisIdx]._pinEndStopMax,
                                            raw._axis[axisIdx]._pinEndStopMaxactLvl);
        }
    }
}
