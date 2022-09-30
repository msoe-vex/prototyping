#include "main.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
using pros::E_CONTROLLER_DIGITAL_R1;

void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);


	pros::Motor motorOne(1, pros::E_MOTOR_GEARSET_36);
	pros::Motor motorTwo(2, pros::E_MOTOR_GEARSET_36);
	pros::Motor motorThree(3, pros::E_MOTOR_GEARSET_36);
	pros::Motor motorOneReversed(4, pros::E_MOTOR_GEARSET_36, true);
	pros::Motor motorTwoReversed(5, pros::E_MOTOR_GEARSET_36, true);
	pros::Motor motorThreeReversed(6, pros::E_MOTOR_GEARSET_36, true);
	pros::Motor miscMotor1(11, pros::E_MOTOR_GEARSET_36);
	pros::Motor miscMotor2(12, pros::E_MOTOR_GEARSET_36);
	pros::Motor miscMotor3(13, pros::E_MOTOR_GEARSET_36);
	pros::Motor miscMotor1Reversed(14, pros::E_MOTOR_GEARSET_36, true);
	pros::Motor miscMotor2Reversed(15, pros::E_MOTOR_GEARSET_36, true);
	pros::Motor miscMotor3Reversed(16, pros::E_MOTOR_GEARSET_36, true);
	pros::Motor_Group motorGroup({motorOne, motorTwo, motorThree, motorOneReversed, motorTwoReversed, motorThreeReversed});
	pros::Motor_Group miscMotorGroup({miscMotor1, miscMotor2, miscMotor3, miscMotor1Reversed, miscMotor2Reversed, miscMotor3Reversed});

	bool analogUsage1 = true; //Determines joystick or button toggle controls
	bool analogUsage2 = true; //Determines joystick or button toggle controls

	bool toggle1 = false;
	bool toggle2 = false;
	int val1 = 60;
	int val2 = 60;

	while (true) {
		//Bad things happening
		//master.print(0,0, "L: %d R: %d     ", round(motorGroup.get_actual_velocities()[0]), round(miscMotorGroup.get_actual_velocities()[0]));
		master.print(1,0,"L: %d R: %d     ", val1, val2);

    	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    	    analogUsage1 = !analogUsage1;
    	}

        if (analogUsage1) { //If analogUsage1 is true, use joystick controls
			motorGroup.move_velocity(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		} else {
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
				toggle1 = !toggle1;
			}

			if (toggle1) {
				motorGroup.move_velocity(val1);
			} else {
				motorGroup.move_velocity(0);
			}
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    	    analogUsage2 = !analogUsage2;
    	}

        if (analogUsage2) { //If analogUsage1 is true, use joystick controls
			miscMotorGroup.move_velocity(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		} else {
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
				toggle2 = !toggle2;
			}
			if (toggle2) {
				miscMotorGroup.move_velocity(val2);
			} else {
				miscMotorGroup.move_velocity(0);
			}
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
			if (val2 == -127) {
				val2 = -126;
			}
			val2 -= 1;
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			if (val2 == 127) {
				val2 = 126;
			}
			val2 += 1;
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			if (val1 == -127) {
				val1 = -126;
			}
			val1 -= 1;
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			if (val1 == 127) {
				val1 = 126;
			}
			val1 += 1;
		}

		// if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
		// 	miscMotorGroup.move(127);
		// } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		// 	miscMotorGroup.move(-127);
		// } else {
		// 	miscMotorGroup.move(0);
		// }

		// delay to not overload the system
		pros::delay(20);
	}

			
		
}

// 2 3-motor-group controlled by toggle (same ports)
// 2 3-motor-groups controlled by a joystick (same ports)
// 1 misc port controller by joystick
// 1 misc port controller by button
