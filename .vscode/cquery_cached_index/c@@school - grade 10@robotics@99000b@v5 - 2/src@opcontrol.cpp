#include "main.h"
#include "okapi/api.hpp"



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
	int power;
	int turn;
	int left;
	int right;
	int buttonToggleState = 0;
	int buttonPressed = 0;
	bool ballstatus=false;
	bool intakeSystem = false;
	bool outtakeSystem = false;
	bool shootingsystem = false;
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor driveleft_B(13);
	pros::Motor driveright_B(18);
	pros::Motor driveleft_T(15);
	pros::Motor driveright_T(17);
	pros::Motor intake(11);
	pros::Motor indexer(20);
	pros::Motor flywheel(12);
	pros::Motor arm(19);
	pros::ADIDigitalIn limit(19);
	pros::ADIGyro gyro(5);
	pros::ADIUltrasonic sensor (7, 8);

gyro.reset();
int balldistance;
bool rpmready;

	while (true) {
		int flywheelrpm = -(flywheel.get_actual_velocity());
		pros::delay(2);

		balldistance = sensor.get_value();

		if (flywheelrpm > (abs(380)))
		{
			rpmready = true;
		}

		else if (flywheelrpm < (abs(380)))
		{
			rpmready = false;
		}
	//	std::cout << "Distance: " << sensor.get_value();
	//	pros::delay(10);

	//	std::cout << "Degrees: " << gyro.get_value();
  // 	pros::delay(10);

		std::cout << "speed: " << rpmready;
   	pros::delay(10);

		 power = master.get_analog(ANALOG_RIGHT_X);
     turn = master.get_analog(ANALOG_RIGHT_Y);
     left = (power * 0.8) + turn;
     right = (power * 0.8) - turn;

if (((abs(power))<5) && ((abs(turn)<5))){
	driveright_B.move_velocity(0);
	driveright_T.move_velocity(0);
	driveleft_B.move_velocity(0);
	driveleft_T.move_velocity(0);
	pros::delay(10);

	driveright_B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveright_T.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveleft_B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveleft_T.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	/*	driveright_B.move_velocity(-20);
		driveright_T.move_velocity(20);
		driveleft_B.move_velocity(20);
		driveleft_T.move_velocity(-20);
		*/
}
 else{
		 driveright_B.move(right);
		 driveright_T.move(right);
		 driveleft_B.move(left);
		 driveleft_T.move(left);
   }

//arm
if(master.get_digital(DIGITAL_L2)) {
arm.move_absolute(400,50);
}

else if (master.get_digital(DIGITAL_UP)) {
 arm.move_absolute(2200,160);
}
else {
arm.move(master.get_analog(ANALOG_LEFT_Y));
}


//flywheel

if (master.get_digital(DIGITAL_RIGHT) ) {
	flywheel.move_velocity(0);
}
else {
	flywheel.move_velocity(-400);
}

//smart intake and indexer
if (master.get_digital(DIGITAL_R2)) {
if ((rpmready = true) && (balldistance < 100)){
	intake.move_velocity(-18);
	indexer.move_velocity(-200);
}
else if ((rpmready = true) && (balldistance > 100)){
	intake.move_velocity(-200);
	indexer.move_velocity(-200);
}
else if ((rpmready = false) && (balldistance < 100)){
	intake.move_velocity(0);
	indexer.move_velocity(0);
}
else if ((rpmready = false) && (balldistance > 100))
{
	intake.move_velocity(-200);
	indexer.move_velocity(0);
}
}

//manual intake
else if (master.get_digital(DIGITAL_L1)) {
 intake.move_velocity(200);
 indexer.move_velocity(0);
}
else if(master.get_digital(DIGITAL_R1)) {
	intake.move_velocity(-200);
	indexer.move_velocity(5);
}

//double shot
else if(master.get_digital(DIGITAL_DOWN)) {
//first ball
	 flywheel.move_velocity(-600);
	 pros::delay(4000);
	 intake.move_velocity(-200);
	 indexer.move_velocity(-200);
	 pros::delay(180);
	 intake.move_velocity(0);
	 indexer.move_velocity(0);
//second ball
	 flywheel.move_velocity(-400);
	 pros::delay(2000);
	 flywheel.move_velocity(-400);
	 intake.move_velocity(-200);
	 indexer.move_velocity(-200);
	 pros::delay(180);
	 intake.move_velocity(0);
	 indexer.move_velocity(0);
 }
 else{
	 intake.move_velocity(0);
	 indexer.move_velocity(0);	 
 }


		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		pros::delay(20);
	}
}
