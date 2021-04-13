#ifndef CAR_H_
#define CAR_H_

#include "image_processing.h"
#include "hardware.h"
#include <cmath>

enum TrackType {
    Straight,
    Corner,
    Intersection
};

class Car {
private:
	bool topUpdated = false;
	bool bottomUpdated = false;
	bool turnChecked = false;
	float steerAmt = 0;
	int endLineFrames = 0;

public:
	float targetSpeedLeft = 0; // between 0 and 1, target speed of left wheel
	float targetSpeedRight = 0; // between 0 and 1, target speed of right wheel
	bool shouldStop = false;
	int nStopFrames = 0;

	int topLeft = -INF; // where the left Line is detected, between 0 and WIDTH
	int topRight= INF; // where the right line is detected, between 0 and WIDTH
	int bottomLeft = -INF; // same as above
	int bottomRight = INF; // same as above

	Car() {}

	void goBrrr() {
        // reset some variables
		topUpdated = false;
		bottomUpdated = false;
		turnChecked = false;

		if (shouldStop) {
			nStopFrames++;
			if (nStopFrames >= targetStopFrames) {
				driveMotor(0);
				return;
			}
		}

        // If on straight line, small adjust only
        switch (getTrackType()) {
            case Straight:
            	if (!shouldStop && checkEndLine()) {
					shouldStop = true;
				}
                straightAdjust();
                enableSingleLed(kMaskLed2);
                break;
            case Corner:
                cornerAdjust();
                enableSingleLed(kMaskLed3);
                break;
            case Intersection:
                intersectionAdjust();
                enableSingleLed(kMaskLed4);
                break;
        }

        turn(steerAmt);
        driveMotorIndividual(targetSpeedLeft, targetSpeedRight, debugMode);
	}

	void updateTop() {
		if (topUpdated) return; // don't update more than once per frame

		getProcessedImage(topRow);
		getLeftRight(topLeft, topRight);
		topUpdated = true;
		debug("Top: %d %d\n", topLeft, topRight);
	}

	void updateBottom() {
		if (bottomUpdated) return; // don't update more than once per frame

		getProcessedImage(bottomRow);
		getLeftRight(bottomLeft, bottomRight);
		bottomUpdated = true;
		debug("Bottom: %d %d\n", bottomLeft, bottomRight);
	}

	int getTrackType() {
        updateTop();

        if(topLeft == -INF && topRight == INF) {
            return Intersection;
        } if (topLeft == -INF || topRight == INF) {
            return Corner;
        }

		return topRight - topLeft > minLRDistance ? Straight : Corner;
	}

	float getSteerAmt(int right, int left) {
        float rightFraction = ((float)(right - MIDDLE)) / ((float)(right - left));
		debug("Right fraction: %d/100\n", (int)(rightFraction * 100));
        if (abs(rightFraction - 0.5f) > maxStraightLineError) {
            // if we deviate enough from the middle, steer
            return rightFraction < 0.5f ? -straightSteerFactor : straightSteerFactor;
        } else {
            return 0.0f;
        }
	}

	void straightAdjust() {
		updateTop();
		steerAmt = getSteerAmt(topRight, topLeft);
		targetSpeedLeft = targetSpeedRight = normalSpeed;
	}

	// returns the amount the car should turn if there's a curve; 0 if there isn't a curve
    void cornerAdjust() {
        updateBottom();

        if (bottomLeft > -INF && bottomRight == INF) {
            // we can see the left line, i.e. right turn
            steerAmt = curveSteerFactor;
            targetSpeedRight = normalSpeed * curveSteerSlowSpeedFactor * curveSpeedFactor;
            targetSpeedLeft = normalSpeed * curveSteerFastSpeedFactor * curveSpeedFactor;
        } else if (bottomLeft == -INF && bottomRight < INF) {
            // we can see the right line, i.e. left turn
            steerAmt = -curveSteerFactor;
            targetSpeedLeft = normalSpeed * curveSteerSlowSpeedFactor * curveSpeedFactor;
            targetSpeedRight = normalSpeed * curveSteerFastSpeedFactor * curveSpeedFactor;
        }
    }

    void intersectionAdjust() {
        steerAmt = 0;
        targetSpeedLeft = targetSpeedRight = normalSpeed * intersectionSpeedFactor;
    }

    bool checkEndLine() {
    	const int lineToCheck = 110;
    	const int colJump = 10;
		const int colStart = MIDDLE - 5 * colJump;
		const int colEnd = MIDDLE + 5 * colJump;
		bool pattern[(colEnd - colStart) / colJump + 2];

		int patternIndex = 0;

		for (int col = colStart; col < colEnd; col += colJump) {
			bool blackRow = false;
			for (int row = lineToCheck - 5; row <= lineToCheck + 5 && !blackRow; ++row) {
				if (!isWhite(getPixel(col, row))) {
					blackRow = true;
				}
			}

			if (patternIndex == 0 || pattern[patternIndex - 1] != blackRow) {
				pattern[patternIndex++] = blackRow;
			}
		}

		int nBlack = 0;
		for (int i = 0; i < patternIndex; ++i) {
			debug("%d", pattern[i]);
			if (pattern[i]) {
				nBlack++;
			}
		}
		debug("\n");

		if (nBlack >= 2) {
			endLineFrames++;
		} else {
			endLineFrames = 0;
		}

		return endLineFrames >= 1;
	}

};

#endif
