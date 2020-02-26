#include "main.h"

extern pros::Controller master;
extern pros::Motor driveleft_B;
extern pros::Motor driveright_B;
extern pros::Motor driveleft_T;
extern pros::Motor driveright_T;
extern pros::Motor intake;
extern pros::Motor indexer;
extern pros::Motor flywheel;
extern pros::Motor arm;
extern pros::ADIDigitalIn limit;
extern pros::ADIGyro gyro;
extern pros::ADIUltrasonic sensor;
extern pros::Task tUpdateActualGyro;
extern double gyroActual;
