#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#define WIDTH 316
#define MIDDLE 158

// speed multiplier for when a curve is detected
const float curveSpeedFactor = 0.75f;
const float curveSteerSlowSpeedFactor = 0.15f;
const float curveSteerFastSpeedFactor = 1.7f;

// speed of the car
float normalSpeed = 0.2f;

// steering multiplier for when the car is going forwards
const float straightSteerFactor = 0.15f;

// steer factor for when we adjust the car position before a curve
const float precurveAdjustSteerFactor = 0.2f;

// speed multiplier for when an intersection is detected
const float intersectionSpeedFactor = 0.7f;

// threshold for when the cars is too much to one side in a straight line
const float maxStraightLineError = 0.05f;

// if the minimum distance between right and left is smaller than this, don't do anything
const int minLRDistance = 40;

// rows camera takes the image from
const int topRow = 10;
const int bottomRow = 100;

// number of frames until we consider we have to turn
const int minCurveFrames = 1;

float curveSteerFactor = 1;

#endif
