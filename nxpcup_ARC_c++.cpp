#include "hardware.h"
#include "car.h"

#define kSpeedTabSize 100

int main() {
	init();
	pixy.init();
	pixy.setLED(0, 255, 0);
	pixy.changeProg("video");

    mDelay_GetDelay(kPit1, 500 / kPit1Period);

    // center servo
    turn(0);

    Car car = Car();

    // main loop
	while(true) {
	    // set debug mode on if switch 4 is set
	    if (readSwitch(kSw4)) {
	        debugMode = true;
	    } else {
	    	debugMode = false;
	    }

	    // cancel if switch 1 is off
	    if (!readSwitch(kSw1)) {
	    	turn(0);
	    	driveMotor(0);
	    	continue;
	    }

	    // toggle leds to know its fps
		toggleLed(kMaskLed1);

		// check if the car needs to adjust the position
		if (car.state == straight) {
			car.checkTurn();
		}

		// if the car is still supposed to go straight
		if (car.state == straight) {
			car.straightLineAdjust();
		} else {
			// update the car's current position
			car.updatePosition();

			// steer towards target if necessary
			car.moveTowardsTarget();
		}

		driveMotor(normalSpeed, debugMode);
	}
}
