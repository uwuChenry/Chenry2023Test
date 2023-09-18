#include "main.h"

void opcontrol() {
	while (true) {
        auto fwd = drive::controller.getAnalog(ControllerAnalog::leftY);
        auto turn = drive::controller.getAnalog(ControllerAnalog::rightX);

		drive::driveChassisPtr->arcade(fwd, turn);
		delay(10);
	}
}
