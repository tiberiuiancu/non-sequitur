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

#define WIDTH 316
#define INF 1000

bool debugMode = false;

#define debug(fmt, ...) \
		if (debugMode) \
        	printf(fmt, ##__VA_ARGS__);

static Int16 sDly;

// values to calibrate the motors and the servo
const float calibrate = 0.965f;
const float servoOffset = 0.085f;

const float minSpeed = 0.35f;
const float maxSpeed = 1.0f;


// buffer should we allocd with size WIDTH + 1
// returns an int array where the first element is the size and the ones to follow are index values where lines were detected
int* getProcessedImage(Pixy2SPI_SS &pixy, const int row, int* buffer) {
	// read image and make greyscale
	int img[WIDTH];
	uint8_t r, g, b;
	for (int i = 0; i < WIDTH; ++i) {
		pixy.video.getRGB(i, row, &r, &g, &b, false);
		img[i] = (r + g + b) / 3;
	}

	// sobel bs
	int sobel[WIDTH];
	sobel[0] = 0;
	sobel[WIDTH - 1] = 0;
	for (int i = 1; i < WIDTH - 1; ++i) {
		sobel[i] = abs(img[i - 1] - img[i + 1]);
	}

	// non-maximum suppresion
	const int filter1 = 8;
	int max_sobel = 0;
	for (int i = 1; i < WIDTH - 1; ++i) {
		if (sobel[i] < sobel[i - 1] || sobel[i] < sobel[i + 1] || sobel[i] < filter1) {
			sobel[i] = 0;
			continue;
		}

		if (sobel[i] > max_sobel) {
			max_sobel = sobel[i];
		}
	}

	// gaussian scaling bs
	const double filter2 = 0.1;
	int line_cnt = 0;
	for (int i = 0; i < WIDTH; ++i) {
		if (sobel[i] > 0) {
			// normalize between 0 and 1
			double x = (double) sobel[i] / max_sobel;

			// apply f(x) = (e^(x^2 + 5x - 1) - e^-1) / e^5
			x = (exp(x * x + 5 * x - 1) - 0.367879441) / 148.413159103;
			if (x > filter2) {
				buffer[++line_cnt] = i;
			}
		}
	}

	buffer[0] = line_cnt;

	return buffer;
}

void getLeftRight(int* lines, int &left, int &right) {
	const int min_line_threshold = 30;
	int line_cnt = lines[0];

	puts("\n");

	left = -1;
	right = INF;

	if (line_cnt > 4) {
		return;
	}

	if (line_cnt == 4) {
		left = lines[2];
		right = lines[3];
	} else if (line_cnt == 3) {
		if (abs(lines[2] - lines[1]) < abs(lines[2] - lines[3])) {
			left = lines[2];
			right = lines[3];
		} else {
			left = lines[1];
			right = lines[2];
		}
	} else if (line_cnt == 2) {
		if (abs(lines[2] - lines[1]) < min_line_threshold) {
			if (lines[2] < WIDTH / 2) {
				left = lines[2];
			} else {
				right = lines[2];
			}
		} else {
			left = lines[1];
			right = lines[2];
		}
	} else {
		if (lines[1] < WIDTH / 2) {
			left = lines[1];
		} else {
			right = lines[1];
		}
	}
}

// Change led state
// led: kMaskLed1, kMaskLed2, kMaskLed3, kMaskLed4
// state: kLedOff, kLedOn
void led(LedMaskEnum led, LedStateEnum state) {
    mLeds_Write(led, state);
}

// led: kMaskLed1, kMaskLed2, kMaskLed3, kMaskLed4
void toggleLed(LedMaskEnum led) {
    mLeds_Toggle(led);
}

void setLights(float val) {
    if (val < -1) val = -1;
    else if (val > 1) val = 1;
    mDac_SetDac0Output((val + 1.0) / 2.0);
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
