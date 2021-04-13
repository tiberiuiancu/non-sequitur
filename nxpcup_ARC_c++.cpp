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


    bool reset = true;

    // main loop
	while(true) {
	    // set debug mode on if switch 4 is set
	    if (readSwitch(kSw4)) {
	        debugMode = true;
	    } else {
	    	debugMode = false;
	    }

	    // cancel if switch 1 is off
        if (!readSwitch(kSw1) && !readSwitch(kSw2) && !readSwitch(kSw3)) {
            turn(0);
            driveMotor(0);
            reset = true;
            continue;
        }

        if (reset) {
            if(readSwitch(kSw1)) initReliable();
            else if(readSwitch(kSw2)) initFastAndFurious();
            else if(readSwitch(kSw3)) initLudicrousSpeed();

            initialSampling();
            reset = false;
        }

	    // toggle leds to know its fps
		toggleLed(kMaskLed1);

		car.goBrrr();
	}
}
