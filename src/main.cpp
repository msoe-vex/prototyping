#include "main.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include "pros/screen.hpp"

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


	pros::Motor motorOne(1);
	pros::Motor motorTwo(2);
	pros::Motor motorThree(3);
	pros::Motor motorOneReversed(4, true);
	pros::Motor motorTwoReversed(5, true);
	pros::Motor motorThreeReversed(6, true);
	pros::Motor miscMotor1(11);
	pros::Motor miscMotor2(12);
	pros::Motor miscMotor3(13);
	pros::Motor miscMotor1Reversed(14, true);
	pros::Motor miscMotor2Reversed(15, true);
	pros::Motor miscMotor3Reversed(16, true);
	pros::Motor_Group motorGroup({motorOne, motorTwo, motorThree, motorOneReversed, motorTwoReversed, motorThreeReversed});
	pros::Motor_Group miscMotorGroup({miscMotor1, miscMotor2, miscMotor3, miscMotor1Reversed, miscMotor2Reversed, miscMotor3Reversed});

	bool analogUsage1 = true; //Determines joystick or button toggle controls for motorGroup
	bool analogUsage2 = true; //Determines joystick or button toggle controls for miscMotorGroup

	/*
	These variables are for usage in toggling the motors with respect to m_state.
	toggle1 : enables/disables state-variable motor intensities for motorGroup.
	toggle2 : enables/disables state-variable motor intensities for miscMotorGroup.
	val1 : a value used by motorGroup to determine how much power it should output.
	val2 : a value used by miscMotorGroup to determine how much power it should output.
	*/
	bool toggle1 = false;
	bool toggle2 = false;
	int val1 = 60;
	int val2 = 60;

	/*
	VelocityState has 3 positive(#), a zero(ZERO), and 3 negative(N_#) intensities 
	and a NONE state to restrict usage.
	*/
	enum VelocityState {
        ONE, TWO, THREE, ZERO, N_ONE, N_TWO, N_THREE, NONE
    };

	//m_state uses VelocityState to determine intensity of motor voltage/speed/power.
	VelocityState m_state = NONE;

	while (true) {
		//Bad things happening
		//master.print(0,0, "L: %d R: %d     ", round(motorGroup.get_actual_velocities()[0]), round(miscMotorGroup.get_actual_velocities()[0]));
		master.print(1,0,"L: %d R: %d     ", val1, val2);

    	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    	    analogUsage1 = !analogUsage1;
    	}

        if (analogUsage1) { //If analogUsage1 is true, use joystick controls for motorGroup
			motorGroup.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		} else {
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
				//When pressing L1, process m_state to the next stage up to THREE
				switch (m_state) { 
					case ONE: {
						m_state = TWO;
					}	
					case TWO: {
						m_state = THREE;
					}	
					case THREE: {
						m_state = THREE;
					}	
					case ZERO: {
						m_state = ONE;
					}
					case N_ONE: {
						m_state = ZERO;
					}	
					case N_TWO: {
						m_state = N_ONE;
					}	
					case N_THREE: {
						m_state = N_TWO;
					}
					case NONE: {
						m_state = ZERO;
					} 
					default: {
						m_state = ZERO;
					}
				}
			}
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
				//When pressing L2, process m_state to the next stage down to N_THREE
				switch (m_state) {
					case ONE: {
						m_state = ZERO;
					}	
					case TWO: {
						m_state = ONE;
					}	
					case THREE: {
						m_state = TWO;
					}	
					case ZERO: {
						m_state = N_ONE;
					}
					case N_ONE: {
						m_state = N_TWO;
					}	
					case N_TWO: {
						m_state = N_THREE;
					}	
					case N_THREE: {
						m_state = N_THREE;
					}
					case NONE: {
						m_state = ZERO;
					} 
					default: {
						m_state = ZERO;
					}
				}
			}
			
			//Enable toggling for motorGroup
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {	
				toggle1 = !toggle1;
			}

			if (toggle1) {
				//move motorGroup based on m_state value
				switch (m_state) {
					case ONE: {
						motorGroup.move(50);
					}	
					case TWO: {
						motorGroup.move(100);
					}	
					case THREE: {
						motorGroup.move(127);
					}	
					case ZERO: {
						motorGroup.move(0);
					}
					case N_ONE: {
						motorGroup.move(-50);
					}	
					case N_TWO: {
						motorGroup.move(-100);
					}	
					case N_THREE: {
						motorGroup.move(-127);
					}
					case NONE: { /*If state-based toggling isn't denoted (NONE), use val1*/
						motorGroup.move(val1);
					}
				} 
			} else {
				motorGroup.move(0);
			}
			//when up/Down are pressed, turn state-based toggling off for digital toggling for motorGroup.
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
				m_state = NONE;
				if (val1 == -127) {
					val1 = -126;
				}
				val1 -= 1;
			} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
				m_state = NONE;
				if (val1 == 127) {
					val1 = 126;
				}
				val1 += 1;
			}
			if (toggle1) {
				//move motorGroup based on m_state value
				switch (m_state) {
					case ONE: {
						motorGroup.move(50);
					}	
					case TWO: {
						motorGroup.move(100);
					}	
					case THREE: {
						motorGroup.move(127);
					}	
					case ZERO: {
						motorGroup.move(0);
					}
					case N_ONE: {
						motorGroup.move(-50);
					}	
					case N_TWO: {
						motorGroup.move(-100);
					}	
					case N_THREE: {
						motorGroup.move(-127);
					}
					case NONE: {
						motorGroup.move(val1);
					}
				} 
			}
		}
		//Enables/disables miscMotorGroup's analog vs digital input
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    	    analogUsage2 = !analogUsage2;
    	}

        if (analogUsage2) { //If analogUsage2 is true, use joystick controls for miscMotorGroup
			miscMotorGroup.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		} else {
			//Enable toggling for miscMotorGroup
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
				toggle2 = !toggle2;
			}
			if (toggle2) {
				miscMotorGroup.move(val2);
			} else {
				miscMotorGroup.move(0);
			}
		}
		//The If, else-if chain below manages the motor power of miscMotorGroup.
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
		//The If, else-if chain below manages the motor power of motorGroup.
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
