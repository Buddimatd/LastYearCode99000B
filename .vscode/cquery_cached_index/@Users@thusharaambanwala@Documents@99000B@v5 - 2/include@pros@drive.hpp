#ifndef DRIVE_H
#define DRIVE_H

void _driveReset();

int drivePos();

bool isDriving();

void driveTask(void* parameter);
void turnTask(void* parameter);

void driveAsync(int sp);
void turnAsync(int sp);
void drive(int sp);
void turn(int sp);
void slowDrive(int sp, int dp = 0);

void setSpeed(int speed);
void setSlant(int s);
void setCurrent(int mA);
void setBrakeMode(int mode);
void driveOp(void* parameter);


#endif
