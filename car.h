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

public:
	float targetSpeedLeft = 0; // between 0 and 1, target speed of left wheel
	float targetSpeedRight = 0; // between 0 and 1, target speed of right wheel

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

        // If on straight line, small adjust only
        switch (getTrackType()) {
            case Straight:
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
        if (topLeft > -INF && topRight < INF && topRight - topLeft > minLRDistance) {
        	return Straight;
        }

        updateBottom();

        if(bottomLeft == -INF && bottomRight == INF) {
            return Intersection;
        }

        if (bottomLeft == -INF || bottomRight == INF) {
            return Corner;
        }

        return Straight;
	}

	float getSteerAmt(int right, int left) {
        float rightFraction = ((float)(right - MIDDLE)) / ((float)(right - left));
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
			targetSpeedRight = normalSpeed * curveSteerSlowSpeedFactor;
			targetSpeedLeft = normalSpeed * curveSteerFastSpeedFactor;
		} else if (bottomLeft == -INF && bottomRight < INF) {
			// we can see the right line, i.e. left turn
			steerAmt = -curveSteerFactor;
			targetSpeedLeft = normalSpeed * curveSteerSlowSpeedFactor;
			targetSpeedRight = normalSpeed * curveSteerFastSpeedFactor;
		}
    }

    void intersectionAdjust() {
        steerAmt = 0;
        targetSpeedLeft = targetSpeedRight = normalSpeed * intersectionSpeedFactor;
    }
};

#endif
