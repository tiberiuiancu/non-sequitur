#ifndef LIB_H
#define LIB_H

extern "C"
{
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"

#include "Modules/mSpi.h"
#include "Modules/mDac.h"
#include "Modules/mAccelMagneto.h"
#include "Modules/mGyro.h"
#include "Modules/mTimer.h"
#include "Modules/mCpu.h"
#include "Modules/mSwitch.h"
#include "Modules/mLeds.h"
#include "Modules/mAd.h"
#include "Modules/mDelay.h"
#include "Modules/mRS232.h"
#include "Modules/mVL6180x.h"

#include "Applications/gInput.h"
#include "Applications/gCompute.h"
#include "Applications/gOutput.h"
}

#include <ctime>
#include <cstdlib>
#include <cmath>
#include "Pixy/Pixy2SPI_SS.h"



#define K_MAIN_INTERVAL	(100 / kPit1Period)
#define kSpeedTabSize 100

// Mesure de vitesse et de sens
static Int16 sDly;

const float calibrate = 0.965f;
const float servoOffset = 0.085f;

const float minSpeed = 0.35f;
const float maxSpeed = 1.0f;

int nSamples = 0;

struct rgb {
    double r, g, b;

    rgb(){}
    rgb(double _r, double _g, double _b) {
        r = _r;
        g = _g;
        b = _b;
    }

    double getMax() {
        return max(max(r, g), b);
    }
} avg;

bool isBlack(rgb c, int threshold = 90) {
    if ((c.r + c.g + c.b) < 3 * threshold)
        return true;
    return false;
}

double cosineSimilarity(rgb a, rgb b) {
    double top = a.r * b.r + a.g * b.g + a.b * b.b;
    double bottom = sqrt(a.r * a.r + a.g * a.g + a.b * a.b) * sqrt(b.r * b.r + b.g * b.g + b.b * b.b);

    // angle * ratio between the smaller and the bigger of the vectors
    return (top / bottom) * ((min(a.getMax(), b.getMax()) / max(a.getMax(), b.getMax())));
}

void addSample(rgb x) {
    if (nSamples == 0) {
        avg.r = x.r;
        avg.g = x.g;
        avg.b = x.b;
    } else {
        avg.r *= ((double) nSamples) / ((double) nSamples + 1);
        avg.g *= ((double) nSamples) / ((double) nSamples + 1);
        avg.b *= ((double) nSamples) / ((double) nSamples + 1);

        avg.r += ((double) x.r) / ((double) nSamples + 1);
        avg.g += ((double) x.g) / ((double) nSamples + 1);
        avg.b += ((double) x.b) / ((double) nSamples + 1);
    }
    nSamples++;
}

void resetSamples() {
    nSamples = 0;
    avg.r = avg.b = avg.g = 0;
}

bool isWhite(rgb x, bool toAdd=false, double threshold=0.85) {
    double c = cosineSimilarity(x, avg);

    if (c >= threshold) {
        if (toAdd) addSample(x);
        return true;
    }
    false;
}


// Change led state
// led: kMaskLed1, kMaskLed2, kMaskLed3, kMaskLed4
// state: kLedOff, kLedOn
void led(LedMaskEnum led, LedStateEnum state) {
    mLeds_Write(led, state);
}
// Togglezzzzzzzzzzzzzzzzzzzzszzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz led
void toggleLed(LedMaskEnum led) {
    mLeds_Toggle(led);
}

// Check if switch is turned on
// sw: kSw1, kSw2, kSw3, kSw4
bool readSwitch(SwitchEnum sw) {
    return mSwitch_ReadSwitch(sw);
}
// but: kPushButSW1, kPushButSW2
bool readButton(PushButEnum but) {
    return mSwitch_ReadPushBut(but);
}

// Turn on the motor
// speed: -1 < speed < 1 and = both motors
void driveMotor(float speed) {
    if (speed < 0.001f) {
        mTimer_SetMotorDuty(0, 0);
    } else {
        speed = speed * (maxSpeed - minSpeed) + minSpeed;
        mTimer_SetMotorDuty(speed * calibrate, speed);
    }
}
// left: -1 < left < 1 and = left motor
// right: -1 < right < 1 and = right motor
void driveMotorIndividual(float left, float right) {
    if (left < 0.001f) {
        left = 0;
    } else {
        left = left * (maxSpeed - minSpeed) + minSpeed;
    }
    if (right < 0.001f) {
        right = 0;
    } else {
        right = right * (maxSpeed - minSpeed) + minSpeed;
    }
    mTimer_SetMotorDuty(left * calibrate, right);
}

// Get the speed of the motor in passed variables
void motorSpeed(float *left, float *right) {
    mTimer_GetSpeed(*&left, *&right);
}
// side: 0 left, 1 right
float motorSpeedSide(int side) {
    float aSpeedMotLeft;
    float aSpeedMotRight;
    mTimer_GetSpeed(&aSpeedMotLeft, &aSpeedMotRight);
    switch (side) {
        case 0:
            return aSpeedMotLeft;
        case 1:
            return aSpeedMotRight;
        default:
            return 0;
    }
}

// Turn the servo
// pos: -1 < pos < 1
void turn(float pos) {
    pos += servoOffset;
    if (pos < -1.0f)
        pos = -1.0f;
    if (pos > 1.0f)
        pos = 1.0f;
    mTimer_SetServoDuty(0, pos);
}

// Read from sensors
// sIMotLeft, sIMotRight: Current going through motor
// sUBatt: Voltage through battery
// kPot1, kPot2: Potentiometers
float readSensor(ADCInputEnum sensor) {
    return mAd_Read(sensor);
}

bool isDelayDone(UInt16 delay) {
   if (mDelay_IsDelayDone(kPit1,sDly)) {
       mDelay_ReStart(kPit1, sDly, delay / kPit1Period);
       return true;
   }
   return false;
}

// Initialize all the sensors
void init() {
    mCpu_Setup();

    // Config and start switches and pushers
    mSwitch_Setup();
    mSwitch_Open();

    // Config and start of LEDs
    mLeds_Setup();
    mLeds_Open();

    // Config and start of ADC
    mAd_Setup();
    mAd_Open();

    // Config and start of SPI
    mSpi_Setup();
    mSpi_Open();

    // Config and start non-blocking delay by PIT
    mDelay_Setup();
    mDelay_Open();

    // Timer Config for Speed Measurement and PWM Outputs for Servos
    mTimer_Setup();
    mTimer_Open();

    // Setup FXOS8700CQ
    mAccelMagneto_Setup();
    mAccelMagneto_Open();

    // Setup FXAS21002C
    mGyro_Setup();
    mGyro_Open();

    // Config and start of the DAC0 used to drive the driver LED lighting
    mDac_Setup();
    mDac_Open();

    // Setup and start of motor and servo PWM controls and speed measurement
    mTimer_Setup();
    mTimer_Open();

    // Enable IRQ at the CPU -> Primask
    __enable_irq();

    // UART 4 monitoring image
    mRs232_Setup();
    mRs232_Open();
}

#endif
