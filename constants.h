#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#define WIDTH 316
#define MIDDLE (int)(WIDTH / 2)

// rows camera takes the image from
int topRow = 25;
int bottomRow = 100;

// if the minimum distance between right and left is smaller than this, don't do anything
const int minLRDistance = 40;

// speed of the car
float normalSpeed = 0.2f;
// speed multiplier for when an intersection is detected
float intersectionSpeedFactor = 0.7f;

// speed multiplier for when a curve is detected
float curveSpeedFactor = 0.5f;
float curveSteerFactor = 0.85f;

const float curveSteerSlowSpeedFactor = 0.75f;
const float curveSteerFastSpeedFactor = 1.25f;

// steering multiplier for when the car is going forwards
const float straightSteerFactor = 0.15f;

// threshold for when the cars is too much to one side in a straight line
const float maxStraightLineError = 0.05f;

void initReliable() {
	bottomRow = 100;
	normalSpeed = 0.2f;
	intersectionSpeedFactor = 0.7f;

	curveSpeedFactor = 0.5f;
	curveSteerFactor = 0.85f;
}

void initFastAndFurious() {
	bottomRow = 90;
	normalSpeed = 0.35f;;
	intersectionSpeedFactor = 0.7f;

	curveSpeedFactor = 0.4f;
	curveSteerFactor = 0.7f;
}

void initLudicrousSpeed() {
	bottomRow= 80;
	normalSpeed = 0.35f;
	intersectionSpeedFactor = 0.8f;

	curveSpeedFactor = 0.35f;
	curveSteerFactor = 0.8f;
}

#endif
