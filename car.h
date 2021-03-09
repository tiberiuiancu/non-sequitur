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
	int curveFrames = 0; // 0 = straight line, < 0 = left turn frames, > 0 = right turn frames
	float steerAmt = 0;

public:
	float targetSpeed = 0; // between 0 and 1, target speed of car
	int topLeft = -INF; // where the left Line is detected, between 0 and WIDTH
	int topRight= INF; // where the right line is detected, between 0 and WIDTH
	int bottomLeft = -INF; // same as above
	int bottomRight = INF; // same as above

	int toTurn = 0; // variable that says whether the car has turned once we have detected a curve ahead; < 0 = left, 0 = don't turn, > 0 = right

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

	void checkTurn() {
		if (topLeft == -INF && topRight < INF) {
			curveFrames--;
		} else if (topLeft > -INF && topRight == INF) {
			curveFrames++;
		} else {
			curveFrames = 0;
		}

		if (abs(curveFrames) >= minCurveFrames) {
			toTurn = curveFrames;
		}

		turnChecked = true;
	}

	bool isStraightLine() {
		checkTurn();
		return curveFrames == 0;
	}

	bool straightLineAdjust() {
		updateTop();

		steerAmt = 0;

		// if we are supposed to turn, don't adjust
		if (!isStraightLine()) {
			return false;
		}

		// if we can't see both lines, return
		if (topLeft == -INF || topRight == INF) {
			return false;
		}

		float rightFraction = ((float)(topRight - MIDDLE)) / ((float)(topRight - topLeft));
		debug("Right fraction: %d/100\n", (int)(rightFraction * 100));
		if (abs(rightFraction - 0.5f) > maxStraightLineError) {
			// if we deviate enmough from the middle, steer
			steerAmt = (rightFraction < 0.5f ? -straightSteerFactor : straightSteerFactor);
		}

		return true;
	}

	// returns the amount the car should turn if there's a curve; 0 if there isn't a curve
	bool curveSteer() {
		checkTurn();
		updateBottom();

		steerAmt = 0;

		// if we haven't detected a curve, don't turn
		if (toTurn == 0) {
			return false;
		}

		// we can see no line; probably intersection
		if (bottomLeft == -INF && bottomRight == INF) {
			return false;
		}

		if (bottomLeft > -INF && bottomRight < INF) {
			// we can see both lines, probably finished a curve
			steerAmt = 0;
		} else if (bottomLeft > -INF && bottomRight == INF) {
			// we can see right turn

			if (toTurn < 0) {
				// probably finished a curve
				steerAmt = 0;
				toTurn = 0; // stop steering afterwards
			} else {
				steerAmt = curveSteerFactor;
			}
		} else if (bottomLeft == -INF && bottomRight < INF) {
			// we can see left turn

			if (toTurn > 0) {
				// probably finished a curve
				steerAmt = 0;
				toTurn = 0; // stop steering afterwards
			} else {
				steerAmt = -curveSteerFactor;
			}
		}

		return true;
	}

	void steer() {
		turn(steerAmt);
	}
};

#endif
