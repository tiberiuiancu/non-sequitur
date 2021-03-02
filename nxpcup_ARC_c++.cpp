#include "lib.h"
#include "car.h"
#include "constants.h"

#define K_MAIN_INTERVAL (100 / kPit1Period)

#define kSpeedTabSize 100

int main() {
	init();

    // Pixycam
    Pixy2SPI_SS pixy;
    pixy.init();
    pixy.setLED(0, 255, 0);

    mDelay_GetDelay(kPit1, 500 / kPit1Period);

    // middle pixel index
    const int middle = WIDTH / 2;

    // variables to store position of left and right lines
    int left = -1;
    int right = INF;

    // center servo
    turn(0);

    // line buffer
	int lines[WIDTH + 1];

    // main loop
	while(true) {
	    // set debug mode on if switch 4 is set
	    if (readSwitch(kSw4)) {
	        debugMode = true;
	    } else {
	    	debugMode = false;
	    }

	    if (readSwitch(kSw1)) {
	    	toggleLed(kMaskLed1);

			getLeftRight(getProcessedImage(pixy, topRow, lines), left, right);
			debug("%d,%d\n", left, right);

            if (left != -1 && right != INF && right - left > minLRDistance) {
                // sees 2 lines
                float rightFraction = ((float)(right - middle)) / ((float)(right - left));
                if (rightFraction > straightThreshold) {
                    turn(straightSteerFactor);
                } else if (rightFraction < 1 - straightThreshold) {
                    turn(-straightSteerFactor);
                } else {
                    turn(0);
                }

				driveMotor(normalSpeed, debugMode);

                continue;
            }

            driveMotor(curveSpeedFactor * normalSpeed, debugMode);
            // if the top view doesn't see 2 lines switch to close view and control curve

            getLeftRight(getProcessedImage(pixy, bottomRow, lines), left, right);

            float leftSpeed = 0;
            float rightSpeed = 0;

			debug("bottom: %d %d\n", left, right);

            if (right - left <= minLRDistance) {
                continue;
            }

            if (right == INF && left == -1) {
                // probably intersection
                leftSpeed = rightSpeed = intersectionSpeedFactor * normalSpeed;
                turn(0);
            } else if (right == INF) {
                // probably right turn
                turn(1);
                rightSpeed = curveSteerSlowSpeedFactor * normalSpeed;
                leftSpeed = curveSteerFastSpeedFactor * normalSpeed;
            } else if (left == -1) {
                // probably left turn
                turn(-1);
                rightSpeed = curveSteerFastSpeedFactor * normalSpeed;
                leftSpeed = curveSteerSlowSpeedFactor *  normalSpeed;
            } else {
                turn(0);
                leftSpeed = rightSpeed = normalSpeed;
            }

			driveMotorIndividual(leftSpeed, rightSpeed, debugMode);
        } else {
	        driveMotor(0);
	        turn(0);
	    }
	}
}
