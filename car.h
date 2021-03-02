#ifndef CAR_H_
#define CAR_H_

#include "image_processing.h"
#include "hardware.h"
#include <cmath>

enum DrivingState {
	straight, prepareTurn
};

class Car {
public:
	float position = 0.5f; // horiontal car position: < 0.5 = it's on the right, > 0.5 = it's on the left, 0.5 = middle
	float target = 0.5f; // horizontal car target: same as above
	float targetSpeed = 0; // between 0 and 1, target speed of car
	int topLeft = -1; // where the left Line is detected, between 0 and WIDTH
	int topRight= INF; // where the right line is detected, between 0 and WIDTH
	int bottomLeft = -1; // same as above
	int bottomRight = INF; // same as above

	int nCurveFrames = 0; // number of frames we see we have to make a curve



	DrivingState state;

	Car() {
		state = straight;
	}

	void checkTurn() {
		getProcessedImage(topRow);
		getLeftRight(topLeft, topRight);
		debug("Top: %d %d\n", topLeft, topRight);

		if (topLeft == -1 && topRight < INF) {
			nCurveFrames++;
			if (nCurveFrames > 3) {
				target = leftSteeringTargetPosition;
				state = prepareTurn;
				pixy.setLED(255, 0, 0);
			}
		} else if (topLeft > 0 && topRight == INF) {
			nCurveFrames++;
			if (nCurveFrames > 3) {
				target = rightSteeringTargetPosition;
				state = prepareTurn;
				pixy.setLED(0, 0, 255);
			}
		} else {
			nCurveFrames = 0;
			state = straight;
			target = 0.5f;
		}
	}

	void updatePosition() {
		getProcessedImage(bottomRow);
		getLeftRight(bottomLeft, bottomRight);

		// calculate proportion thing
		if (bottomLeft > 0 && bottomRight < INF) {
			// we can see both lines
			position = ((float)(bottomRight - MIDDLE)) / ((float)(bottomRight - bottomLeft));
		} else if (bottomLeft > 0) {
			// we can see the left line
			position = (float) bottomLeft / MIDDLE;
		} else if (bottomRight < INF) {
			position = (float) (bottomRight - MIDDLE) / (WIDTH - MIDDLE);
		} else {
			position = 0.5f;
		}

		debug("Car position: %d/100\n", (int) (position * 100));
	}

	void straightLineAdjust() {
		float rightFraction = ((float)(topRight - MIDDLE)) / ((float)(topRight - topLeft));

		if (abs(rightFraction - target) < maxStraightLineError) {
			turn(0);
		} else {
			turn(rightFraction < target ? -straightSteerFactor : straightSteerFactor);
		}
	}

	void moveTowardsTarget() {
		if (abs(target - position) < maxPositionError) {
			turn(0);
		} else {
			turn(position < target ? -precurveAdjustSteerFactor : precurveAdjustSteerFactor);
		}
	}
};

#endif
