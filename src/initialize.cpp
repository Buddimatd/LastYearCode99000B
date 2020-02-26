#include "config.hpp"
#include "main.h"

bool mirror = false;

void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

void check(void* param);
void autointake(void* param);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

bool rpmready;
int flywheelrpm;

void initialize() {
    // prosv5 upload --slot 4
    // prosv5 v5 rm-file slot_4.bin --erase-all

    gyro.reset();
    // pros::Task checkball(check, (void*)"PROS", TASK_PRIORITY_DEFAULT,
    // TASK_STACK_DEPTH_DEFAULT, "checkball"); pros::Task autointakes(autointake,
    // (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
    // "autointake");

    pros::lcd::initialize();
    pros::lcd::set_text(1, "THUSHARA JUULS");

    pros::lcd::register_btn1_cb(on_center_button);

    if (flywheelrpm > (abs(380))) {
        rpmready = true;
    }

    else if (flywheelrpm < (abs(380))) {
        rpmready = false;
    }

    pros::delay(2000);

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

/*
void check (void* param) {
        pros::ADIUltrasonic ultrasonic (8, 7);
        bool ballstatus=false;

 while(true) {
         if((ultrasonic.get_value())>100){
                 ballstatus=true;
         }
 else {
         ballstatus=false;
 }
 }
}



void autointake (void* param) {
while(true) {
        bool ballstatus=false;
  bool intakeSystem = false;
  bool outtakeSystem = false;
 pros::Motor intake(11);
 pros::Motor indexer(20);

//intake balls
                while (intakeSystem==true) {
                        //for intaking (not shooting
                                //no ball behind indexer
                                if (ballstatus==false) {
                                        //run both indexer and intake
                                        intake.move_velocity(200);
                                        indexer.move_velocity(60);
                                }
                                //ball behind indexer
                                else if (ballstatus==true) {
                                        //turn off indexer and only run intake
                                        intake.move_velocity(200);
                                        indexer.move_velocity(0);
                                }

                                        if (ballstatus==false) {
                                                intake.move_velocity(120);
                                                indexer.move_velocity(200);
                                        }
                                else if (ballstatus==true) {
                                        intake.move_velocity(0);
                                        indexer.move_velocity(200);
                                }
                        }

                //do not intake balls
                while (intakeSystem==false)
                {
                        if(outtakeSystem==true) {
                                intake.move_velocity(200);
                                } else {
                                        intake.move_velocity(0);

                        }
                        pros::delay(20);
                }
        }
}
*/
