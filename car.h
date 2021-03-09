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

		if (abs(curveFrames) > minCurveFrames) {
			toTurn = curveFrames;
		}

		turnChecked = true;
	}

//	void updatePosition() {
//		getProcessedImage(bottomRow);
//		getLeftRight(bottomLeft, bottomRight);
//
//		// calculate proportion thing
//		if (bottomLeft > 0 && bottomRight < INF) {
//			// we can see both lines
//			position = ((float)(bottomRight - MIDDLE)) / ((float)(bottomRight - bottomLeft));
//		} else if (bottomLeft > 0) {
//			// we can see the left line
//			position = (float) bottomLeft / MIDDLE;
//		} else if (bottomRight < INF) {
//			position = (float) (bottomRight - MIDDLE) / (WIDTH - MIDDLE);
//		} else {
//			position = 0.5f;
//		}
//
//		debug("Car position: %d/100\n", (int) (position * 100));
//	}

	bool isStraightLine() {
		checkTurn();
		return abs(curveFrames) > 0;
	}

	float straightLineAdjust() {
		updateTop();

		// if we are supposed to turn, don't adjust
		if (!isStraightLine()) {
			return 0;
		}

		// if we can't see both lines, return
		if (topLeft == -INF || topRight == INF) {
			return 0;
		}

		float rightFraction = ((float)(topRight - MIDDLE)) / ((float)(topRight - topLeft));

		if (abs(rightFraction - 0.5f) < maxStraightLineError) {
			return 0;
		} else {
			return (rightFraction < 0.5f ? -straightSteerFactor : straightSteerFactor);
		}
	}

	// returns the amount the car should turn if there's a curve; 0 if there isn't a curve
	float curveTurnAmount() {
		checkTurn();
		updateBottom();

		// if we haven't detected a curve, don't turn
		if (toTurn == 0) {
			return 0;
		}

		if (bottomLeft > -INF && bottomRight < INF) {
			// we can see both lines, don't turn
			return 0;
		}

		// we can see no line; probably intersection
		if (bottomLeft == -INF && bottomRight == INF) {
			return 0;
		}

		// we see a right turn
		if (bottomLeft > -INF && bottomRight == INF) {
			if (toTurn < 0) {
				// prior information says to turn left; do nothing for now
				return 0;
			}

			return curveSteerFactor;
		}

		if (bottomLeft == -INF && bottomRight < INF) {
			if (toTurn > 0) {
				// prior information says to turn right; do nothing for now
				return 0;
			}

			return -curveSteerFactor;
		}
	}
};

#endif
