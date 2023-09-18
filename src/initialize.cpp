#include "main.h"

int AutonNum = 0;

void printAutonSelect(int num){
	switch (num)
	{
	case 0:
		pros::lcd::print(3, "leftAuton");
		break;
	case 1:
		pros::lcd::print(3, "rightAuton");
		break;
	case 2:
		pros::lcd::print(3, "rightMidAuton");
		break;
	case 3:
		pros::lcd::print(3, "soloAWP");
		break;
	case 4:
		pros::lcd::print(3, "skillsNoPark");
		break;
	case 5:
		pros::lcd::print(3, "skillsWithPark");
		break;
	default:
		break;
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void leftButton(){
	bool pressed = false;
	pressed = !pressed;
  	if (pressed) {
    	pros::lcd::set_text(2, "I was pressed!");
		AutonNum --;
		AutonNum = std::clamp(AutonNum, 0, 5);
		printAutonSelect(AutonNum);
  	}
	else {
    	pros::lcd::clear_line(2);
  	}
}


void centerButton(){
	pros::lcd::print(3, "gyroResettingggggggg");
	delay(1000);
	drive::resetGyro();
	drive::waitUntilGyroRest();
	pros::lcd::print(2, "gyroReset");
}

void rightButton(){
	bool pressed = false;
	pressed = !pressed;
  	if (pressed) {
    	pros::lcd::set_text(2, "I was pressed!");
		AutonNum ++;
		AutonNum = std::clamp(AutonNum, 0, 5);
		printAutonSelect(AutonNum);
  	}
	else {
    	pros::lcd::clear_line(2);
  	}
}


void initialize() {
	pros::lcd::initialize();
	delay(500);
	pros::lcd::register_btn0_cb(leftButton);
	pros::lcd::register_btn2_cb(rightButton);
	pros::lcd::register_btn1_cb(centerButton);
	drive::resetMotorEncoder();
	pros::lcd::set_text(2, "finish init");
	printf("finish init");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
