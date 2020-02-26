#include "okapi/api.hpp"

using namespace okapi::literals;

okapi::Motor motorDriveLeftFront{2_mtr};
okapi::Motor motorDriveLeftBack{1_mtr};
okapi::Motor motorDriveRightFront{4_rmtr};
okapi::Motor motorDriveRightBack{3_rmtr};

okapi::MotorGroup motorDriveLeft{{motorDriveLeftBack, motorDriveLeftFront}};
okapi::MotorGroup motorDriveRight{{motorDriveRightFront, motorDriveRightBack}};

okapi::MotorGroup motorDriveAll{{motorDriveLeftBack, motorDriveLeftFront,
                                 motorDriveRightBack, motorDriveRightFront}};
