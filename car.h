#ifndef CAR_H_
#define CAR_H_

#include "image_processing.h"
#include "hardware.h"
#include <cmath>

class Car {
private:
	bool topUpdated = false;
	bool bottomUpdated = false;
	bool turnChecked = false;
	float steerAmt = 0;

public:
	float targetSpeed = 0; // between 0 and 1, target speed of car
	int topLeft = -INF; // where the left Line is detected, between 0 and WIDTH
	int topRight= INF; // where the right line is detected, between 0 and WIDTH
	int bottomLeft = -INF; // same as above
	int bottomRight = INF; // same as above

	Car() {}

	void startFrame() {
		topUpdated = false;
		bottomUpdated = false;
		turnChecked = false;
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

	bool isStraightLine() {
        updateTop();

        // if we can't see both lines, return
        if (topLeft == -INF || topRight == INF) {
            return false;
        } else if(topLeft == -INF && topRight == INF) {
        	return false;
        }

		return topRight - topLeft > minLRDistance;
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

	void straightLineAdjust() {
		updateTop();
		steerAmt = getSteerAmt(topRight, topLeft);
	}

	// returns the amount the car should turn if there's a curve; 0 if there isn't a curve
	void computeCurveSteer() {
		updateBottom();

		if (bottomLeft > -INF && bottomRight == INF) {
			// we can see the left line, i.e. right turn
			steerAmt = curveSteerFactor;
		} else if (bottomLeft == -INF && bottomRight < INF) {
			// we can see the right line, i.e. left turn
			steerAmt = -curveSteerFactor;
		}
	}

	void steer() {
		turn(steerAmt);
	}
};

#endif
