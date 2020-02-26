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

 pros::Controller master(pros::E_CONTROLLER_MASTER);
 pros::Motor driveleft_B(13, false);
 pros::Motor driveright_B(18, false);
 pros::Motor driveleft_T(15, true);
 pros::Motor driveright_T(17, true);
 pros::Motor intake(11);
 pros::Motor indexer(20);
 pros::Motor flywheel(12);
 pros::Motor arm(19);
 pros::ADIGyro gyro(5);
 pros::ADIUltrasonic ultrasonic (8, 7);

/*
 auto drive = okapi::ChassisControllerFactory::create(
     {driveleft_T, driveleft_B}, {driveright_T, driveright_B},
     okapi::AbstractMotor::gearset::green,
     const ChassisScales &amp;iscales = ChassisScales({4_in, 11.5_in}) )

 );
*/
 int currentgyro = gyro.get_value();

 void leftturn(int turnvalue,int velo,int time) {
   driveleft_B.move_absolute(-turnvalue,velo);
   driveleft_T.move_absolute(-turnvalue,velo);
   driveright_B.move_absolute(-turnvalue,velo);
   driveright_T.move_absolute(-turnvalue,velo);

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

 void rightturn(int rightvalue,int velo, int time) {
   driveleft_B.move_absolute(rightvalue,velo);
   driveleft_T.move_absolute(rightvalue,velo);
   driveright_B.move_absolute(rightvalue,velo);
   driveright_T.move_absolute(rightvalue,velo);

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

 void rightturn2(int rightvalue,int velo, int time) {
   driveleft_B.move_absolute(rightvalue,0);
   driveleft_T.move_absolute(rightvalue,0);
   driveright_B.move_absolute(rightvalue,velo);
   driveright_T.move_absolute(rightvalue,velo);

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

void impforward(int forvalue, int velo){

int currdrivevalue=(driveleft_B.get_position());

if (currdrivevalue<(-forvalue)){
  driveleft_B.move_velocity(velo);
  driveleft_T.move_velocity(velo);
  driveright_B.move_velocity(-velo);
  driveright_T.move_velocity(-velo);
}
else
{
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


 void forward(int forvalue,int velo, int time) {
driveleft_B.move_absolute(forvalue,velo);
driveleft_T.move_absolute(forvalue,velo);
driveright_B.move_absolute(-forvalue,velo);
driveright_T.move_absolute(-forvalue,velo);

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

 void back(int backvalue,int velo, int time) {
   driveleft_B.move_absolute(-backvalue,velo);
   driveleft_T.move_absolute(-backvalue,velo);
   driveright_B.move_absolute(backvalue,velo);
   driveright_T.move_absolute(backvalue,velo);

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
  //driveright_B.set_brake_mode(MOTOR_BRAKE_HOLD);
  //driveright_T.set_brake_mode(MOTOR_BRAKE_HOLD);
  //driveleft_B.set_brake_mode(MOTOR_BRAKE_HOLD);
  //driveleft_T.set_brake_mode(MOTOR_BRAKE_HOLD);
  driveleft_B.tare_position();
  driveleft_T.tare_position();
  driveright_T.tare_position();
  driveright_B.tare_position();
}

 void armup(int liftvalue,int velo) {
arm.move_absolute(liftvalue,velo);
 }

 void armdown(int liftvalue,int velo) {
arm.move_absolute(-liftvalue,velo);
 }


void stoparm(int time) {
  pros::delay(time);
  arm.move_velocity(0);
}

void armpid (int value, int velo, int time) {
  arm.move_absolute(value,velo);
  pros::delay(time);
  arm.move_velocity(200);
  pros::delay(time);
  arm.move_velocity(0);
}

 void intakes(int intakevalue) {
intake.move_velocity(-intakevalue);
 }

void stopintake(int time) {
  pros::delay(time);
  intake.move_velocity(0);
}
 void indexers(int indexervalue) {
indexer.move_velocity(indexervalue);
 }

 void stopindexer(int time) {
   pros::delay(time);
   indexer.move_velocity(0);
 }

 void bothintake(int bothvalue,int time){
   indexer.move_velocity(bothvalue);
   intake.move_velocity(bothvalue);
   pros::delay(time);
   intake.move_velocity(0);
   indexer.move_velocity(0);
 }

 void flywheels(int velo) {
flywheel.move_velocity(velo);
 }

void stopflywheel(int time) {
  pros::delay(time);
  flywheel.move_velocity(0);
}

 void doubleshot(int shootvalue,int time, int endtime) {
flywheel.move_velocity(shootvalue);
pros::Task::delay_until(&now,2000);
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

void drivemypid(int distance, int speed)
{
  //if (fabs(0 - driveleft_B.get_position()) < 20){
while((driveleft_B.get_position()) < distance && (-driveright_B.get_position()) < distance)
{
driveright_B = -speed;
driveright_T = -speed;
driveleft_B = speed;
driveleft_T = speed;
pros::delay(20);
  }

  driveright_B = (speed)/9;
  driveright_T = (speed)/9;
  driveleft_B = (-speed)/9;
  driveleft_T = (-speed)/9;

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

void revdrivemypid(int distance, int speed)
{
  //if (fabs(0 - driveleft_B.get_position()) < 20){
while((fabs(driveleft_B.get_position())) < distance && (fabs(-driveright_B.get_position())) < distance)
{
driveright_B = speed;
driveright_T = speed;
driveleft_B = -speed;
driveleft_T = -speed;
pros::delay(20);
  }

  driveright_B = (-speed)/9;
  driveright_T = (-speed)/9;
  driveleft_B = (speed)/9;
  driveleft_T = (speed)/9;

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

while(true){
  //int current = ((abs(driveleft_B.get_current_draw())) + (abs(driveright_B.get_current_draw())) + (abs(driveleft_T.get_current_draw())) + (abs(driveright_T.get_current_draw())))/4;
int current = (driveleft_B.get_current_draw());

//int position = ((fabs(driveleft_B.get_position())) + (fabs(driveright_B.get_position())) + (abs(driveleft_T.get_current_draw())) + (abs(driveright_T.get_current_draw())))/4;
int position = (driveleft_B.get_position());

if (position < distance)
{
  driveleft_B.move_velocity(currentspeed);
  driveleft_T.move_velocity(currentspeed);
  driveright_B.move_velocity(-currentspeed);
  driveright_T.move_velocity(-currentspeed);
}
 else{
 pros::delay(time);
   driveleft_B.move_velocity(0);
   driveleft_T.move_velocity(0);
   driveright_B.move_velocity(0);
   driveright_T.move_velocity(0);
   //driveright_B.set_brake_mode(MOTOR_BRAKE_HOLD);
   //driveright_T.set_brake_mode(MOTOR_BRAKE_HOLD);
   //driveleft_B.set_brake_mode(MOTOR_BRAKE_HOLD);
   //driveleft_T.set_brake_mode(MOTOR_BRAKE_HOLD);
}
}

driveleft_B.tare_position();
driveleft_T.tare_position();
driveright_T.tare_position();
driveright_B.tare_position();

}

void forward2(int forvalue,int velo, int time) {

  driveleft_B.move_relative(forvalue,velo);
  driveleft_T.move_relative(forvalue,velo);
  driveright_B.move_relative(-forvalue,velo);
  driveright_T.move_relative(-forvalue,velo);

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

void reverse2 (int forvalue,int velo, int time) {

  driveleft_B.move_relative(-forvalue,velo);
  driveleft_T.move_relative(-forvalue,velo);
  driveright_B.move_relative(forvalue,velo);
  driveright_T.move_relative(forvalue,velo);

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

void gyroturn (int turnvalue, int velo){

  int currentgyro = gyro.get_value();


  if (currentgyro < turnvalue){
  driveleft_B.move_velocity(velo);
  driveleft_T.move_velocity(velo);
  driveright_B.move_velocity(velo);
  driveright_T.move_velocity(velo);
}
else {
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

void positiveTurn(int angle, int speed)
{
	gyro.reset();
	while ((fabs(gyro.get_value())) < angle)
	{
		driveleft_B = speed;
		driveleft_T = speed;
		driveright_B = speed;
		driveright_T = speed;
    pros::delay(20);
	}
  driveleft_B = (-speed)/4;
	driveleft_T = (-speed)/4;
	driveright_B = (-speed)/4;
	driveright_T = (-speed)/4;

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

void negativeTurn(int angle, int speed)
{
	gyro.reset();
	while ((fabs(gyro.get_value())) < angle)
	{
		driveleft_B = -speed;
		driveleft_T = -speed;
		driveright_B = -speed;
		driveright_T = -speed;
    pros::delay(20);
	}
  driveleft_B = (speed)/3;
	driveleft_T = (speed)/3;
	driveright_B = (speed)/3;
	driveright_T = (speed)/3;

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

void newrightturn(int rightvalue,int velo) {

  driveleft_B.move_absolute(rightvalue,velo);
  driveleft_T.move_absolute(rightvalue,velo);
  driveright_B.move_absolute(rightvalue,velo);
  driveright_T.move_absolute(rightvalue,velo);

    driveleft_B.tare_position();
    driveleft_T.tare_position();
    driveright_T.tare_position();
    driveright_B.tare_position();
}

void autonomous() {
//Blue side front (Shoot flags and flip cap)
/*
flywheels(-430);
intakes(200);
drivemypid(2400, 80);
stopintake(100);
pros::delay(300);
revdrivemypid(2700,80);
positiveTurn(870, 30);
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
//Blue back (Shoot middle flag and Stack cap)
/*
flywheels(-365);
intakes(200);
indexers(5);
drivemypid(2360, 100);
stopintake(500);
revdrivemypid(1360, 60);
pros::delay(300);
positiveTurn(670,30);
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

//Red side front (3 flags and flip cap)
/*
flywheels(-430);
intakes(200);
drivemypid(2400, 80);
stopintake(100);
pros::delay(300);
revdrivemypid(2700,80);
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

//Red side front (3 flags and platform)
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

//Blue side front (3 flags and platform)
/*
flywheels(-430);
intakes(200);
drivemypid(2400, 80);
stopintake(500);
revdrivemypid(2800,80);
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

//Red back (Cap and platform)
/*
flywheels(-365);
intakes(200);
drivemypid(2400, 160);
stopintake(500);
back(1350, 120, 1500);
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
revdrivemypid(2300, 160);
armup(0, 180);
positiveTurn(910,120);
forwardtime(200,1600);
*/

//Blue back (Cap and platform)

flywheels(-365);
intakes(200);
drivemypid(2400, 160);
stopintake(500);
back(1350, 120, 1500);
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


}
