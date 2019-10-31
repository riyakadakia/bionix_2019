#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftDriveSmart = motor(PORT2, ratio18_1, false);
motor RightDriveSmart = motor(PORT1, ratio18_1, true);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 130, mm, 1);
motor RightClawMotor = motor(PORT3, ratio18_1, false);
motor ArmMotor = motor(PORT6, ratio36_1, true);
motor Stacker = motor(PORT7, ratio36_1, false);
motor LeftClawMotor = motor(PORT8, ratio18_1, true);
motor RightFrontBaseMotor = motor(PORT9, ratio18_1, false);
motor LeftFrontBaseMotor = motor(PORT10, ratio18_1, false);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}