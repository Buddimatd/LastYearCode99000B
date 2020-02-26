#include "config.hpp"
#include "main.h"

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

std::uint32_t now = pros::millis();
/*
 auto drive = okapi::ChassisControllerFactory::create(
     {driveleft_T, driveleft_B}, {driveright_T, driveright_B},
     okapi::AbstractMotor::gearset::green,
     const ChassisScales &amp;iscales = ChassisScales({4_in, 11.5_in}) )

 );
*/
int currentgyro = gyro.get_value();
bool isTurnTaskActive = false;

void leftturn(int turnvalue, int velo, int time) {
    driveleft_B.move_absolute(-turnvalue, velo);
    driveleft_T.move_absolute(-turnvalue, velo);
    driveright_B.move_absolute(-turnvalue, velo);
    driveright_T.move_absolute(-turnvalue, velo);

    pros::delay(time);
    driveleft_B.move_velocity(0);
    driveleft_T.move_velocity(0);
    driveright_B.move_velocity(0);
    driveright_T.move_velocity(0);

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void rightturn(int rightvalue, int velo, int time) {
    driveleft_B.move_absolute(rightvalue, velo);
    driveleft_T.move_absolute(rightvalue, velo);
    driveright_B.move_absolute(rightvalue, velo);
    driveright_T.move_absolute(rightvalue, velo);

    pros::delay(time);
    driveleft_B.move_velocity(0);
    driveleft_T.move_velocity(0);
    driveright_B.move_velocity(0);
    driveright_T.move_velocity(0);

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void rightturn2(int rightvalue, int velo, int time) {
    driveleft_B.move_absolute(rightvalue, 0);
    driveleft_T.move_absolute(rightvalue, 0);
    driveright_B.move_absolute(rightvalue, velo);
    driveright_T.move_absolute(rightvalue, velo);

    pros::delay(time);
    driveleft_B.move_velocity(0);
    driveleft_T.move_velocity(0);
    driveright_B.move_velocity(0);
    driveright_T.move_velocity(0);

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void impforward(int forvalue, int velo) {
    int currdrivevalue = (driveleft_B.get_position());

    if (currdrivevalue < (-forvalue)) {
        driveleft_B.move_velocity(velo);
        driveleft_T.move_velocity(velo);
        driveright_B.move_velocity(-velo);
        driveright_T.move_velocity(-velo);
    } else {
        driveleft_B.move_velocity(0);
        driveleft_T.move_velocity(0);
        driveright_B.move_velocity(0);
        driveright_T.move_velocity(0);

        driveleft_B.tare_position();
        driveleft_T.tare_position();
        driveright_T.tare_position();
        driveright_B.tare_position();
    }
}

void forward(int forvalue, int velo, int time) {
    driveleft_B.move_absolute(forvalue, velo);
    driveleft_T.move_absolute(forvalue, velo);
    driveright_B.move_absolute(-forvalue, velo);
    driveright_T.move_absolute(-forvalue, velo);

    pros::delay(time);
    driveleft_B.move_velocity(0);
    driveleft_T.move_velocity(0);
    driveright_B.move_velocity(0);
    driveright_T.move_velocity(0);

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void forwardtime(int forvalue, int time) {
    driveleft_B.move_velocity(forvalue);
    driveleft_T.move_velocity(forvalue);
    driveright_B.move_velocity(-forvalue);
    driveright_T.move_velocity(-forvalue);

    pros::delay(time);
    driveleft_B.move_velocity(0);
    driveleft_T.move_velocity(0);
    driveright_B.move_velocity(0);
    driveright_T.move_velocity(0);

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void back(int backvalue, int velo, int time) {
    driveleft_B.move_absolute(-backvalue, velo);
    driveleft_T.move_absolute(-backvalue, velo);
    driveright_B.move_absolute(backvalue, velo);
    driveright_T.move_absolute(backvalue, velo);

    pros::delay(time);
    driveleft_B.move_velocity(0);
    driveleft_T.move_velocity(0);
    driveright_B.move_velocity(0);
    driveright_T.move_velocity(0);

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void drivestop(int time) {
    pros::delay(time);
    driveleft_B.move_velocity(0);
    driveleft_T.move_velocity(0);
    driveright_B.move_velocity(0);
    driveright_T.move_velocity(0);
    // driveright_B.set_brake_mode(MOTOR_BRAKE_HOLD);
    // driveright_T.set_brake_mode(MOTOR_BRAKE_HOLD);
    // driveleft_B.set_brake_mode(MOTOR_BRAKE_HOLD);
    // driveleft_T.set_brake_mode(MOTOR_BRAKE_HOLD);
    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void armup(int liftvalue, int velo) { arm.move_absolute(liftvalue, velo); }

void armdown(int liftvalue, int velo) { arm.move_absolute(-liftvalue, velo); }

void stoparm(int time) {
    pros::delay(time);
    arm.move_velocity(0);
}

void armpid(int value, int velo, int time) {
    arm.move_absolute(value, velo);
    pros::delay(time);
    arm.move_velocity(200);
    pros::delay(time);
    arm.move_velocity(0);
}

void intakes(int intakevalue) { intake.move_velocity(-intakevalue); }

void stopintake(int time) {
    pros::delay(time);
    intake.move_velocity(0);
}
void indexers(int indexervalue) { indexer.move_velocity(indexervalue); }

void stopindexer(int time) {
    pros::delay(time);
    indexer.move_velocity(0);
}

void bothintake(int bothvalue, int time) {
    indexer.move_velocity(bothvalue);
    intake.move_velocity(bothvalue);
    pros::delay(time);
    intake.move_velocity(0);
    indexer.move_velocity(0);
}

void flywheels(int velo) { flywheel.move_velocity(velo); }

void stopflywheel(int time) {
    pros::delay(time);
    flywheel.move_velocity(0);
}

void doubleshot(int shootvalue, int time, int endtime) {
    flywheel.move_velocity(shootvalue);
    pros::Task::delay_until(&now, 2000);
    intake.move_velocity(200);
    indexer.move_velocity(200);
    pros::delay(endtime);
    flywheel.move_velocity(0);
    intake.move_velocity(0);
    indexer.move_velocity(0);
}

void stopforward(int velo) {
    driveleft_B.move_velocity(velo);

    driveleft_T.move_velocity(velo);
    driveright_B.move_velocity(velo);
    driveright_T.move_velocity(velo);
}

void drivemypid(int distance, int speed) {
    // if (fabs(0 - driveleft_B.get_position()) < 20){
    while ((driveleft_B.get_position()) < distance &&
           (-driveright_B.get_position()) < distance) {
        driveright_B = -speed;
        driveright_T = -speed;
        driveleft_B = speed;
        driveleft_T = speed;
        pros::delay(20);
    }

    driveright_B = (speed) / 9;
    driveright_T = (speed) / 9;
    driveleft_B = (-speed) / 9;
    driveleft_T = (-speed) / 9;

    pros::delay(250);

    driveright_B = 0;
    driveright_T = 0;
    driveleft_B = 0;
    driveleft_T = 0;

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void revdrivemypid(int distance, int speed) {
    // if (fabs(0 - driveleft_B.get_position()) < 20){
    while ((fabs(driveleft_B.get_position())) < distance &&
           (fabs(-driveright_B.get_position())) < distance) {
        driveright_B = speed;
        driveright_T = speed;
        driveleft_B = -speed;
        driveleft_T = -speed;
        pros::delay(20);
    }

    driveright_B = (-speed) / 9;
    driveright_T = (-speed) / 9;
    driveleft_B = (speed) / 9;
    driveleft_T = (speed) / 9;

    pros::delay(250);

    driveright_B = 0;
    driveright_T = 0;
    driveleft_B = 0;
    driveleft_T = 0;

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void currentforward(int distance, int currentspeed, int time) {
    while (true) {
        // int current = ((abs(driveleft_B.get_current_draw())) +
        // (abs(driveright_B.get_current_draw())) +
        // (abs(driveleft_T.get_current_draw())) +
        // (abs(driveright_T.get_current_draw())))/4;
        int current = (driveleft_B.get_current_draw());

        // int position = ((fabs(driveleft_B.get_position())) +
        // (fabs(driveright_B.get_position())) +
        // (abs(driveleft_T.get_current_draw())) +
        // (abs(driveright_T.get_current_draw())))/4;
        int position = (driveleft_B.get_position());

        if (position < distance) {
            driveleft_B.move_velocity(currentspeed);
            driveleft_T.move_velocity(currentspeed);
            driveright_B.move_velocity(-currentspeed);
            driveright_T.move_velocity(-currentspeed);
        } else {
            pros::delay(time);
            driveleft_B.move_velocity(0);
            driveleft_T.move_velocity(0);
            driveright_B.move_velocity(0);
            driveright_T.move_velocity(0);
            // driveright_B.set_brake_mode(MOTOR_BRAKE_HOLD);
            // driveright_T.set_brake_mode(MOTOR_BRAKE_HOLD);
            // driveleft_B.set_brake_mode(MOTOR_BRAKE_HOLD);
            // driveleft_T.set_brake_mode(MOTOR_BRAKE_HOLD);
        }
    }

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void forward2(int forvalue, int velo, int time) {
    driveleft_B.move_relative(forvalue, velo);
    driveleft_T.move_relative(forvalue, velo);
    driveright_B.move_relative(-forvalue, velo);
    driveright_T.move_relative(-forvalue, velo);

    pros::delay(time);
    driveleft_B.move_velocity(0);
    driveleft_T.move_velocity(0);
    driveright_B.move_velocity(0);
    driveright_T.move_velocity(0);

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void reverse2(int forvalue, int velo, int time) {
    driveleft_B.move_relative(-forvalue, velo);
    driveleft_T.move_relative(-forvalue, velo);
    driveright_B.move_relative(forvalue, velo);
    driveright_T.move_relative(forvalue, velo);

    pros::delay(time);
    driveleft_B.move_velocity(0);
    driveleft_T.move_velocity(0);
    driveright_B.move_velocity(0);
    driveright_T.move_velocity(0);

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void gyroturn(int turnvalue, int velo) {
    int currentgyro = gyro.get_value();

    if (currentgyro < turnvalue) {
        driveleft_B.move_velocity(velo);
        driveleft_T.move_velocity(velo);
        driveright_B.move_velocity(velo);
        driveright_T.move_velocity(velo);
    } else {
        driveleft_B.move_velocity(0);
        driveleft_T.move_velocity(0);
        driveright_B.move_velocity(0);
        driveright_T.move_velocity(0);

        driveleft_B.tare_position();
        driveleft_T.tare_position();
        driveright_T.tare_position();
        driveright_B.tare_position();
    }
}

void positiveTurn(int angle, int speed) {
    gyro.reset();
    while ((fabs(gyro.get_value())) < angle) {
        driveleft_B = speed;
        driveleft_T = speed;
        driveright_B = speed;
        driveright_T = speed;
        pros::delay(20);
    }
    driveleft_B = (-speed) / 4;
    driveleft_T = (-speed) / 4;
    driveright_B = (-speed) / 4;
    driveright_T = (-speed) / 4;

    pros::delay(250);

    driveleft_B = 0;
    driveleft_T = 0;
    driveright_B = 0;
    driveright_T = 0;

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void negativeTurn(int angle, int speed) {
    gyro.reset();
    while ((fabs(gyro.get_value())) < angle) {
        driveleft_B = -speed;
        driveleft_T = -speed;
        driveright_B = -speed;
        driveright_T = -speed;
        pros::delay(20);
    }
    driveleft_B = (speed) / 3;
    driveleft_T = (speed) / 3;
    driveright_B = (speed) / 3;
    driveright_T = (speed) / 3;

    pros::delay(250);

    driveleft_B = 0;
    driveleft_T = 0;
    driveright_B = 0;
    driveright_T = 0;

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void newrightturn(int rightvalue, int velo) {
    driveleft_B.move_absolute(rightvalue, velo);
    driveleft_T.move_absolute(rightvalue, velo);
    driveright_B.move_absolute(rightvalue, velo);
    driveright_T.move_absolute(rightvalue, velo);

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

#define MAX 200;

static int driveMode = 1;
static int driveTarget = 0;
static int turnTarget = 0;
static int maxSpeed = MAX;
static int slant = 0;

/*********************************/
// Basic Controls
// Controls left side of chassis
void Chasis_left(int value) {
    driveleft_B.move(value);
    driveleft_T.move(value);
}

// Controls right side of chassis
void Chasis_right(int value) {
    driveright_B.move(value);
    driveright_T.move(value);
}

// Clears the Motors encoders
void _driveClr() {
    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_B.tare_position();
    driveright_T.tare_position();
}

// Resets the whole drive system
void _driveReset() {
    maxSpeed = MAX;
    slant = 0;
    driveTarget = 0;
    _driveClr();
    Chasis_left(0);
    Chasis_right(0);
}

// if called returns integer value
int drivePos() {
    return (driveleft_B.get_position() + driveright_B.get_position()) / 2;
}

// set the max speed for slew control
void setSpeed(int speed) { maxSpeed = speed; }

/*********************************/
// slew control

// 9 works fine if any other number does not work
const int accel_step = 4;
const int deccel_step = 120;  // no decel slew (256)
static int leftSpeed = 0;
static int rightSpeed = 0;

void leftSlew(int leftTarget) {
    int step;

    if (abs(leftSpeed) < abs(leftTarget))
        step = accel_step;
    else
        step = deccel_step;

    if (leftTarget > leftSpeed + step)
        leftSpeed += step;
    else if (leftTarget < leftSpeed - step)
        leftSpeed -= step;
    else
        leftSpeed = leftTarget;

    Chasis_left(leftSpeed);
}

// slew control
void rightSlew(int rightTarget) {
    int step;

    if (abs(rightSpeed) < abs(rightTarget))
        step = accel_step;
    else
        step = deccel_step;

    if (rightTarget > rightSpeed + step)
        rightSpeed += step;
    else if (rightTarget < rightSpeed - step)
        rightSpeed -= step;
    else
        rightSpeed = rightTarget;

    Chasis_right(-rightSpeed);
}

/**************************************************/
// slop correction
void slop(int sp) {
    driveMode = 2;
    if (sp < 0) {
        Chasis_right(-30);
        pros::delay(100);
    }
    driveMode = 1;
}

/**************************************************/
// feedback
bool isDriving() {
    static int count = 0;
    static int last = 0;
    static int lastTarget = 0;

    int leftPos = driveleft_B.get_position();
    int rightPos = driveright_B.get_position();

    int curr = (abs(leftPos) + abs(rightPos)) / 2;
    int thresh = 3;
    int target = turnTarget;

    if (driveMode == 1) target = driveTarget;

    if (abs(last - curr) < thresh)
        count++;
    else
        count = 0;

    if (target != lastTarget) count = 0;

    lastTarget = target;
    last = curr;

    // not driving if we haven't moved
    if (count > 4)
        return false;
    else
        return true;
}

/**************************************************/
// autonomous functions
void driveAsync(int sp) {
    _driveReset();
    driveTarget = sp;
    driveMode = 1;
}

void turnAsync(int sp) {
    if (mirror) sp = -sp;  // inverted turn for blue auton
    _driveReset();
    turnTarget = sp;
    isTurnTaskActive = true;
    driveMode = 0;
}

void drive(int sp) {
    driveAsync(sp);
    pros::delay(450);
    while (isDriving()) pros::delay(20);
}

void turn(int sp) {
    turnAsync(sp);
    pros::delay(450);
    while (isDriving()) pros::delay(20);
}

void slowDrive(int sp, int dp) {
    driveAsync(sp);

    if (sp > 0)
        while (drivePos() < dp) pros::delay(20);
    else
        while (drivePos() > dp) pros::delay(20);

    setSpeed(60);
    while (isDriving()) pros::delay(20);
}

/**************************************************/
// drive modifiers
void setSlant(int s) {
    if (mirror) s = -s;

    slant = s;
}

void setCurrent(int mA) {
    driveleft_B.set_current_limit(mA);
    driveleft_T.set_current_limit(mA);
    driveright_B.set_current_limit(mA);
    driveright_T.set_current_limit(mA);
}

void setBrakeMode(int mode) {
    pros::motor_brake_mode_e_t brakeMode;
    switch (mode) {
        case 0:
            brakeMode = MOTOR_BRAKE_COAST;
            break;
        case 1:
            brakeMode = MOTOR_BRAKE_BRAKE;
            break;
        case 2:
            brakeMode = MOTOR_BRAKE_HOLD;
            break;
    }

    driveleft_B.set_brake_mode(brakeMode);
    driveleft_T.set_brake_mode(brakeMode);
    driveright_B.set_brake_mode(brakeMode);
    driveright_T.set_brake_mode(brakeMode);
}
/**************************************************/
// task control
void driveTask(void* parameter) {
    int prevError = 0;

    while (1) {
        pros::delay(20);

        if (driveMode != 1) continue;

        int sp = driveTarget;

        double kp = .3;
        double kd = .5;

        // read sensors
        int ls = driveleft_B.get_position();
        int rs = driveright_B.get_position();
        int sv = ls;

        // speed
        int error = sp - sv;
        int derivative = error - prevError;
        prevError = error;
        int speed = error * kp + derivative * kd;

        if (speed > maxSpeed) speed = maxSpeed;
        if (speed < -maxSpeed) speed = -maxSpeed;

        // set motors
        leftSlew(speed - slant);
        rightSlew(speed + slant);
    }
}

double getActualGyro() {
    double gyroFinal = gyro.get_value();
    double gyroInitial = 0;
    double static gyroActual;

    while (true) {
        gyroInitial = gyroFinal;
        pros::delay(20);
        gyroFinal = gyro.get_value();

        if (gyroFinal - gyroInitial > 100) {
            gyroActual += (gyroFinal - gyroInitial - 3600) * 0.942408376963351;
        } else if (gyroFinal - gyroInitial < -100) {
            gyroActual += (gyroFinal - gyroInitial + 3600) * 0.942408376963351;
        } else {
            gyroActual += (gyroFinal - gyroInitial) * 0.942408376963351;
        }

        if (gyroActual >= 3600)
            gyroActual -= 3600;
        else if (gyroActual <= -3600)
            gyroActual += 3600;

        return gyroActual;
    }
}

void turnTask(void* parameter) {
    double prevError;
    double errorDifference;
    double errorSum = 0;
    double error = turnTarget - gyroActual;
    int target;

    double kp = 0.3;
    double ki = 0.00;
    double kd = 0;

    double power;

    while (true) {
        error = turnTarget - gyroActual;
        errorSum = 0;
        errorDifference = 0;

        std::cout << "Error: " << error << std::endl;

        while ((fabs(error)) > 20 && isTurnTaskActive) {
            std::cout << "Error: " << error << std::endl;
            prevError = error;
            error = turnTarget - gyroActual;
            errorSum = error + errorSum;
            errorDifference = error - prevError;

            if (errorSum * ki > 127.0) errorSum = 127.0 / ki;
            else if (errorSum * ki < -127.0) errorSum = -127.0 / ki;

            if (abs(error) < 20) errorSum = 0;

            power = kp * error + ki * errorSum - kd * errorDifference;

            Chasis_left(power);
            Chasis_right(power);

            pros::delay(20);
        }

        isTurnTaskActive = false;
        pros::delay(20);
    }
}
void autonomous() {
    std::cout << "Degrees: " << int(turnTarget);

    // Blue front (Shoot flags and flip cap)

    _driveReset();  // reset the drive encoders
    pros::Task drive_task(driveTask);
    pros::Task turn_task(turnTask);

    turn(900);
    // Start Writing code here
    /*
    flywheels(-430);
    intakes(100);
    drive(2500);
    intakes(200);
    stopintake(400);
    drive(-2600);
  */

    // End code here
    drive_task.remove();
    _driveReset();  // make sure no motors are moving
    /*
      positiveTurn(800, 30);

      drivemypid(1400,80);

      indexers(-200);
      intakes(100);
      stopintake(200);
      stopindexer(500);
      drivemypid(920,80);
      intakes(200);
      indexers(-200);
      stopintake(500);
      stopindexer(500);
      revdrivemypid(200,200);
      rightturn(130, 200, 150);
      drivemypid(300,200);
      revdrivemypid(1100, 120);
      intakes(-100);
      negativeTurn(900, 40);
      drivemypid(2000, 200);

    */

    // Red back (Shoot middle flag and Stack cap)

    /*_driveReset(); // reset the drive encoders
      pros::Task drive_task(driveTask);
      pros::Task turn_task(turnTask);

    flywheels(-365);
    intakes(200);
    indexers(5);
    drive(2360);
    stopintake(500);
    drive(-1360);
    pros::delay(300);

    // End code here
      drive_task.remove();
      turn_task.remove();
      _driveReset(); // make sure no motors are moving

    negativeTurn(570,30);
    intakes(200);
    indexers(-200);
    stopintake(1000);
    stopindexer(1000);
    negativeTurn(540,50);
    revdrivemypid(1750, 70);
    pros::delay(600);
    armup(600,50);
    pros::delay(1000);
    drivemypid(450, 60);
    negativeTurn(300, 60);
    drivemypid(2200, 80);
    armup(2200,180);
    pros::delay(1000);
    revdrivemypid(400, 120);
    armup(0, 180);
    positiveTurn(900,60);*/

    // Blue back (Shoot middle flag and Stack cap)
    /*
    _driveReset(); // reset the drive encoders
      pros::Task drive_task(driveTask);
      pros::Task turn_task(turnTask);

    flywheels(-365);
    intakes(200);
    indexers(5);
    drive(2360);
    stopintake(500);
    drive(-1360);
    pros::delay(300);

    // End code here
      drive_task.remove();
      turn_task.remove();
      _driveReset(); // make sure no motors are moving

    positiveTurn(570,30);
    intakes(200);
    indexers(-200);
    stopintake(1000);
    stopindexer(1000);
    positiveTurn(540,50);
    revdrivemypid(1750, 70);
    pros::delay(600);
    armup(600,50);
    pros::delay(1000);
    drivemypid(450, 60);
    positiveTurn(300, 60);
    drivemypid(2200, 80);
    armup(2200,180);
    pros::delay(1000);
    revdrivemypid(400, 120);
    armup(0, 180);
    negativeTurn(900,60);
    */

    // Red side front (3 flags and flip cap)
    /*
    _driveReset(); // reset the drive encoders
      pros::Task drive_task(driveTask);
      pros::Task turn_task(turnTask);

    flywheels(-430);
    intakes(200);
    drive(2400);
    stopintake(100);
    pros::delay(300);
    drive(-2700);

    // End code here
      drive_task.remove();
      turn_task.remove();
      _driveReset(); // make sure no motors are moving

    negativeTurn(850, 30);
    drivemypid(1460,80);
    indexers(-200);
    intakes(100);
    stopintake(200);
    stopindexer(500);
    drivemypid(920,80);
    intakes(200);
    indexers(-200);
    stopintake(500);
    stopindexer(500);
    revdrivemypid(200,200);
    negativeTurn(30, 150);
    drivemypid(500,200);
    revdrivemypid(1300, 120);
    intakes(-100);
    positiveTurn(900, 40);
    drivemypid(2000, 200);
    */

    // Red side front (3 flags and platform)
    /*
    flywheels(-430);
    intakes(200);
    drivemypid(2400, 80);
    stopintake(100);
    pros::delay(300);
    revdrivemypid(2700,80);
    negativeTurn(880, 30);
    drivemypid(1460,80);
    indexers(-200);
    intakes(100);
    stopintake(200);
    stopindexer(500);
    drivemypid(920,80);
    intakes(200);
    indexers(-200);
    stopintake(500);
    stopindexer(500);
    revdrivemypid(200,200);
    negativeTurn(30, 150);
    drivemypid(500,200);
    revdrivemypid(4600, 120);
    positiveTurn(900, 40);
    armup(0, 50);
    forwardtime(200, 4000);
    */

    // Blue side front (3 flags and platform)
    /*
    _driveReset(); // reset the drive encoders
      pros::Task drive_task(driveTask);
      pros::Task turn_task(turnTask);

    flywheels(-430);
    intakes(200);
    drive(2400);
    stopintake(500);
    drive(-2800);

    // End code here
      drive_task.remove();
      turn_task.remove();
      _driveReset(); // make sure no motors are moving

    positiveTurn(880, 30);
    drivemypid(1460,80);
    indexers(-200);
    intakes(200);
    stopintake(200);
    stopindexer(500);
    drivemypid(920,80);
    intakes(200);
    indexers(-200);
    stopintake(500);
    stopindexer(500);
    drivemypid(700,100);
    revdrivemypid(4900, 160);
    negativeTurn(900, 40);
    armup(0, 50);
    forwardtime(200, 4000);
    */

    // Red back (Cap and platform)
    /*
    _driveReset(); // reset the drive encoders
      pros::Task drive_task(driveTask);
      pros::Task turn_task(turnTask);

    flywheels(-365);
    intakes(200);
    drive(2400);
    stopintake(500);
    drive(-1350);

    // End code here
      drive_task.remove();
      turn_task.remove();
      _driveReset(); // make sure no motors are moving

    leftturn(1000,150,1000);
    revdrivemypid(1750, 60);
    pros::delay(300);
    armup(670,50);
    pros::delay(300);
    drivemypid(450, 60);
    negativeTurn(330, 50);
    drivemypid(2350, 100);
    armup(2200,180);
    pros::delay(1000);
    revdrivemypid(2450, 160);
    armup(0, 180);
    positiveTurn(910,120);
    forwardtime(200,1600);
    */

    // Blue back (Cap and platform)
    /*
    _driveReset(); // reset the drive encoders
      pros::Task drive_task(driveTask);
      pros::Task turn_task(turnTask);

    flywheels(-365);
    intakes(200);
    drive(2400);
    stopintake(500);
    drive(-1350);

    // End code here
      drive_task.remove();
      turn_task.remove();
      _driveReset(); // make sure no motors are moving

    rightturn(1000,150,1000);
    revdrivemypid(1750, 60);
    pros::delay(300);
    armup(670,50);
    pros::delay(300);
    drivemypid(450, 60);
    positiveTurn(450, 50);
    drivemypid(2420, 100);
    armup(2200,180);
    pros::delay(1000);
    revdrivemypid(2620, 160);
    armup(0, 180);
    negativeTurn(890,120);
    armup(0, 50);
    forwardtime(200,1400);
    */
}
