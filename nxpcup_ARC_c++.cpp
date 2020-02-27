#include "lib.h"

#define WIDTH 316
#define INF 1000

#define K_MAIN_INTERVAL (100 / kPit1Period)

#define kSpeedTabSize 100

int main() {
	init();

    // Pixycam
    Pixy2SPI_SS pixy;
    pixy.init();
    pixy.setLED(0, 255, 0);

    mDelay_GetDelay(kPit1, 500 / kPit1Period);


    uint8_t r[316], g[316], b[316];

    // middle pixel index
    const int middle = WIDTH / 2;

    // speed of the car
    float normalSpeed = 0.20f;

    // steering multiplier for when the car is going forwards
    const float straightSteerFactor = 0.4f;

    // speed multiplier for when a curve is detected
    const float curveSpeedFactor = 0.5f;
    const float curveSpeedWheelFactor = 0.5f;

    // speed multiplier for when an intersection is detected
    const float intersectionSpeedFactor = 0.8f;

    // if the minimum distance between right and left is smaller than this, don't do anything
    const int minLRDistance = 10;

    // row camera takes the image from
    const int row = 100;

    bool debugMode = false;
    bool reset = true;
    bool toturn = false;

    turn(0);
    mDac_SetDac0Output((mAd_Read(kPot1) + 1.0) / 2.0);
//    pixy.setLamp(100, 100);
	while(true) {
	    if (readSwitch(kSw4)) {
	        debugMode = true;
	    } else {
	    	debugMode = false;
	    }

	    if (readSwitch(kSw1)) {
            if (reset) {
                resetSamples();
                for (int i = middle - 2; i < middle + 2; ++i) {
                    pixy.video.getRGB(i, row, r + i, g + i, b + i, false);
                    addSample({r[i], g[i], b[i]});
                }

                reset = false;
            }

            pixy.setLED(255, 0, 0);
            int left = -1;
            int right = INF;
            float speed = 0;
            for (int i = middle - 1; i >= 0; --i) {
                pixy.video.getRGB(i, row, r + i, g + i, b + i, false);
                if (!isWhite({r[i], g[i], b[i]}, middle - i < 10)) {
                    left = i;
                    break;
                }
            }

            for (int i = middle; i < WIDTH; i++) {
                pixy.video.getRGB(i, row, r + i, g + i, b + i, false);
                if (!isWhite({r[i], g[i], b[i]}, i - middle < 10)) {
                    right = i;
                    break;
                }
            }

//            printf("%d %d\n", left, right);
//            printf("%d %d %d\n", r[middle], g[middle], b[middle]);
//            printf("%d\n", cosineSimilarity({r[middle], g[middle], b[middle]}, avg) * 100);
            if (right - left <= minLRDistance) {
                continue;
            }

            normalSpeed = readSensor(kPot1);
            curveSpeedFactor = readSensor(kPot2);
            toturn = false;

            if (right == INF && left == -1) {
                // probably intersection
                speed = intersectionSpeedFactor * normalSpeed;
                turn(0);
            } else if (right == INF) {
                // probably right turn
                turn(1);
                toturn = true;
                leftspeed = curveSpeedFactor * normalSpeed;
                rightspeed = curveSpeedFactor * curveSpeedWheelFactor * normalSpeed;
            } else if (left == -1) {
                // probably left turn
                turn(-1);
                toturn = true;
                leftspeed = curveSpeedFactor * curveSpeedWheelFactor * normalSpeed;
                rightspeed = curveSpeedFactor * normalSpeed;
            } else {
                turn(0);
                speed = normalSpeed;
            }

            if (debugMode) {
                driveMotor(0);
            } else {
                if (toturn) {
                    driveMotorIndividual(leftspeed, rightspeed);
                } else {
                    driveMotor(speed);
                }
            }
        } else {
	        reset = true;

	        driveMotor(0);
	        turn(0);
	    }
	}

	return 0;
}
