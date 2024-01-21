/**
 * Line Follower project.
 * 
 * This is the program for a simple Line follower using PID control.
 * It requires and Arduino UNO R3, 2 DC motors, L293D motor driver and QTR-8A 6 reflectance sensors array.
 * 
 * Created 20 Jan 2024
 * By Tudor-David Butufei, Tudor Coriciuc, Ana-Iulia Staci
 */
#include <QTRSensors.h>


// Motor pins
const int m21Pin = 7;
const int m22Pin = 6;
const int m11Pin = 5;
const int m12Pin = 4;

const int m2Enable = 11;
const int m1Enable = 10;

// Motor configuration
const int maxSpeed = 255;
const int minSpeed = -255;
const int baseSpeed = 200;
const int boostSpeedThreshold = 100;
const int boostSpeedValue = 20;

// Sensor configuration
QTRSensors qtr;
const int sensorCount = 6;
const uint8_t sensorPins[sensorCount] = {A0, A1, A2, A3, A4, A5};

// Calibration configuration
const int calibrationSpeed = 160;
const int maxCalibrationValue = 50;
const int calibrationCenterDelta = 5;

// Data intervals
const int minSensorsValue = 0;
const int maxSensorsValue = 5000;
const int minErrorValue = -50;
const int maxErrorValue = 50;

// Timing configurations
const int sensorSetupDelay = 500;
const int mainRoutineStartDelay = 2000;

// PID constants
const float kp = 7.75;
const float ki = 0;
const float kd = 30;

// PID values
int p = 1;
int i = 0;
int d = 0;

int error = 0;
int lastError = 0;

// Program state
uint16_t sensorValues[sensorCount];

int m1Speed = 0;
int m2Speed = 0;


/**
 * @brief Setup line follower pins and calibrate sensor.
 */
void setup()
{
    // pinMode setup
    pinMode(m11Pin, OUTPUT);
    pinMode(m12Pin, OUTPUT);
    pinMode(m21Pin, OUTPUT);
    pinMode(m22Pin, OUTPUT);
    pinMode(m1Enable, OUTPUT);
    pinMode(m2Enable, OUTPUT);

    pinMode(LED_BUILTIN, OUTPUT);
   
    // setup sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins(sensorPins, sensorCount);
    delay(sensorSetupDelay);
    
    // calibrate sensor
    calibration();

    delay(mainRoutineStartDelay);
}


/**
 * @brief Continuously calculate PID error and update motor speeds.
 */
void loop()
{
    // calculate motor speeds using PID
    pidControl(kp, ki, kd);

    // update motor speeds based on values computed by PID
    setMotorSpeed(m1Speed, m2Speed);
}


/**
 * @brief Calibrate the QTR sensors my moving the linefollower to the left and to the right of the line.
 */
void calibration()
{
    // turn on Arduino's LED to indicate we are in calibration mode
    digitalWrite(LED_BUILTIN, HIGH);

    // alternate left - right while calibrating
    int dir = 1;
    for (uint16_t i = 0; i < 400; i++)
    {
        qtr.calibrate();

        // set motor speed based on current movement direction (right or left)
        setMotorSpeed(-dir * calibrationSpeed, dir * calibrationSpeed);

        // change movement direction if reached outside the line
        error = map(qtr.readLineBlack(sensorValues), minSensorsValue, maxSensorsValue, minErrorValue, maxErrorValue);
        if (error >= maxCalibrationValue && dir == 1) {
            dir = -1;
        } else if (error <= -maxCalibrationValue && dir == -1) {
            dir = 1;
        }
    }
    setMotorSpeed(0, 0);
    
    // move back to center
    while(error < -calibrationCenterDelta|| error > calibrationCenterDelta) {
        setMotorSpeed(-dir * calibrationSpeed, dir * calibrationSpeed);
        error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);
        if (error >= calibrationCenterDelta && dir == 1) {
            dir = -1;
        } else if (error <= -calibrationCenterDelta && dir == -1) {
            dir = 1;
        }
    }
    setMotorSpeed(0, 0);

    // callibration finished
    digitalWrite(LED_BUILTIN, LOW);
}


/**
 * @brief Calculate PID value based on error, kp, kd, ki, p, i and d.
 * 
 * @param kp Kp constant in PID computation
 * @param ki Ki constant in PID computation
 * @param kd Kd constant in PID computation
 */
void pidControl(float kp, float ki, float kd)
{
    // read error from sensor
    int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);

    // compute PID components
    p = error;
    i = i + error;
    d = error - lastError;

    // pid motor speed
    int motorSpeed = kp * p + ki * i + kd * d;

    // compute speed for each motor
    m1Speed = baseSpeed;
    m2Speed = baseSpeed;

    if (motorSpeed < 0)
    {
        m1Speed += motorSpeed;
    }
    else if (motorSpeed > 0)
    {
        m2Speed -= motorSpeed;
    }

    // boost when single motor running
    if (m1Speed < boostSpeedThreshold && m1Speed > -boostSpeedThreshold) {
        m2Speed += boostSpeedValue;
    }

    if (m2Speed < boostSpeedThreshold && m2Speed > -boostSpeedThreshold) {
        m1Speed += boostSpeedValue;
    }

    // make sure it doesn't go past motor speed limits
    m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
    m2Speed = constrain(m2Speed, minSpeed, maxSpeed);

    lastError = error;
}

/**
 * @brief Set the speed for each motor.
 * Accepted values are between -255 and 255 (negative values mean reverse).
 * 
 * @param motor1Speed the speed value for the first motor
 * @param motor2Speed the speed value for the second motor
 */
void setMotorSpeed(int motor1Speed, int motor2Speed)
{
    if (motor1Speed == 0)
    {
        digitalWrite(m11Pin, LOW);
        digitalWrite(m12Pin, LOW);
        analogWrite(m1Enable, motor1Speed);
    }
    else
    {
        if (motor1Speed > 0)
        {
            digitalWrite(m11Pin, HIGH);
            digitalWrite(m12Pin, LOW);
            analogWrite(m1Enable, motor1Speed);
        }
        if (motor1Speed < 0)
        {
            digitalWrite(m11Pin, LOW);
            digitalWrite(m12Pin, HIGH);
            analogWrite(m1Enable, -motor1Speed);
        }
    }
    if (motor2Speed == 0)
    {
        digitalWrite(m21Pin, LOW);
        digitalWrite(m22Pin, LOW);
        analogWrite(m2Enable, motor2Speed);
    }
    else
    {
        if (motor2Speed > 0)
        {
            digitalWrite(m21Pin, HIGH);
            digitalWrite(m22Pin, LOW);
            analogWrite(m2Enable, motor2Speed);
        }
        if (motor2Speed < 0)
        {
            digitalWrite(m21Pin, LOW);
            digitalWrite(m22Pin, HIGH);
            analogWrite(m2Enable, -motor2Speed);
        }
    }
}
