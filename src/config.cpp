#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor driveleft_B(13, pros::E_MOTOR_GEARSET_18, false,
                        pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveright_B(18, pros::E_MOTOR_GEARSET_18, false,
                         pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveleft_T(15, pros::E_MOTOR_GEARSET_18, true,
                        pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveright_T(17, pros::E_MOTOR_GEARSET_18, true,
                         pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intake(11, pros::E_MOTOR_GEARSET_18, true,
                   pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor indexer(20, pros::E_MOTOR_GEARSET_18, true,
                    pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor flywheel(12, pros::E_MOTOR_GEARSET_06, true,
                     pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor arm(19, pros::E_MOTOR_GEARSET_18, true,
                pros::E_MOTOR_ENCODER_COUNTS);
pros::ADIGyro gyro(5);
pros::ADIUltrasonic sensor(7, 8);

double gyroActual = 0;

void updateActualGyro(void* parameter) {
    double gyroFinal = gyro.get_value();
    double gyroInitial = 0;

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

        std::cout << "Actual: " << gyroActual << std::endl;
    }
}

pros::Task tUpdateActualGyro{updateActualGyro};

