#ifndef CAR_H_
#define CAR_H_

#include "image_processing.h"
#include "hardware.h"
#include <cmath>

enum DrivingState {
	straight, prepareLeftTurn, prepareRightTurn
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

	DrivingState state;

	Car() {
		state = straight;
	}

	bool checkTurn() {
		getProcessedImage(topRow);
		getLeftRight(topLeft, topRight);
		if (topLeft == -1 && topRight < INF) {
			// left turn
			state = prepareLeftTurn;
			target = leftSteeringTargetPosition;
			return true;
		} else if (topLeft > 0 && topRight == INF) {
			state = prepareRightTurn;
			target = rightSteeringTargetPosition;
			return true;
		}

		state = straight;
		target = 0.5f;
		return false;
	}

	void updatePosition() {
		getProcessedImage(bottomRow);
		getLeftRight(bottomLeft, bottomRight);

		// calculate proportion thing
		if (bottomLeft > 0 && bottomRight < INF) {
			// we can see both lines
			position = ((float)(bottomRight - MIDDLE)) / (bottomRight - bottomLeft);
		} else if (bottomLeft > 0) {
			// we can see the left line
			position = (float) bottomLeft / bottomLeftLineMax;
		} else if (bottomRight < INF) {
			position = (float) (bottomRight - bottomRightLineMin) / (WIDTH - bottomRightLineMin);
		} else {
			position = 0.5f;
		}

		debug("Car position: %d/100\n", (int) (position * 100));
	}

	void moveTowardsTarget() {
		if (abs(target - position) < maxPositionError) {
			return;
		}

		turn(position < target ? precurveAdjustSteerFactor : -precurveAdjustSteerFactor);
	}
};

#endif
