#include "lib.h"


#define K_MAIN_INTERVAL (100 / kPit1Period)

#define kSpeedTabSize 100

int main() {
	init();

    // Pixycam
    Pixy2SPI_SS pixy;
    pixy.init();
    pixy.setLED(0, 255, 0);

    mDelay_GetDelay(kPit1, 500 / kPit1Period);

    uint8_t r, g, b;

    // middle pixel index
    const int middle = WIDTH / 2;

    // speed of the car
    float normalSpeed = 0.25f;

    // steering multiplier for when the car is going forwards
    const float straightSteerFactor = 0.2f;

    // speed multiplier for when a curve is detected
    const float curveSpeedFactor = 1.0f;
    const float curveSteerSpeedFactor = 0.5f;

    // speed multiplier for when an intersection is detected
    const float intersectionSpeedFactor = 0.8f;
    
    // threshold for when the cars is too much to one side in a straight line
    const float straightThreshold = 0.7f;

    // if the minimum distance between right and left is smaller than this, don't do anything
    const int minLRDistance = 10;

    // rows camera takes the image from
    const int topRow = 50;
    const int bottomRow = 100;

    // control variables
    bool debugMode = false;
    bool reset = true;

    // variables to store position of left and right lines
    int left = -1;
    int right = INF;

    // center servo
    turn(0);

    // main loop
	while(true) {
	    // set debug mode on if switch 4 is set
	    if (readSwitch(kSw4)) {
	        debugMode = true;
	    } else {
	    	debugMode = false;
	    }

	    if (readSwitch(kSw1)) {
	        // add initial color samples
            if (reset) {
                resetSamples();
                for (int i = middle - 2; i < middle + 2; ++i) {
                    pixy.video.getRGB(i, bottomRow, &r, &g, &b, false);
                    addSample({r, g, b});
                }
                reset = false;
            }

            // check top row for straight lines
            left = getLeft(pixy, topRow);
            right = getRight(pixy, bottomRow);

            if (left != -1 && right != INF) {
                // sees 2 lines
                float rightFraction = ((float)(right - middle)) / ((float)(right - left));
                if (rightFraction > straightThreshold) {
                    turn(straightSteerFactor);
                } else if (rightFraction < 1 - straightThreshold) {
                    turn(-straightSteerFactor);
                } else {
                    turn(0);
                }

                driveMotor(normalSpeed);
                continue;
            }

            // if the top view doesn't see 2 lines switch to close view and control curve
            left = getLeft(pixy, bottomRow);
            right = getRight(pixy, bottomRow);

            float leftSpeed = 0;
            float rightSpeed = 0;

            if (debugMode) {
                printf("%d %d\n", left, right);
            }

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
                rightSpeed = curveSteerSpeedFactor * curveSpeedFactor * normalSpeed;
                leftSpeed = curveSpeedFactor * normalSpeed;
            } else if (left == -1) {
                // probably left turn
                turn(-1);
                rightSpeed = curveSpeedFactor * normalSpeed;
                leftSpeed = curveSteerSpeedFactor * curveSpeedFactor * normalSpeed;
            } else {
                turn(0);
                leftSpeed = rightSpeed = normalSpeed;
            }

            if (debugMode) {
                driveMotor(0);
            } else {
                driveMotorIndividual(leftSpeed, rightSpeed);
            }
        } else {
	        reset = true;

	        driveMotor(0);
	        turn(0);
	    }
	}
}
