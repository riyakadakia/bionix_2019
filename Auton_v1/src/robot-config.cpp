#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor RightClawMotor = motor(PORT3, ratio18_1, false);
motor ArmMotor = motor(PORT6, ratio36_1, true);
motor StackerMotor = motor(PORT7, ratio36_1, false);
motor LeftClawMotor = motor(PORT8, ratio18_1, true);
motor RightFrontMotor = motor(PORT9, ratio18_1, true);
motor LeftFrontMotor = motor(PORT10, ratio18_1, false);
motor RightBackMotor = motor(PORT1, ratio18_1, true);
motor LeftBackMotor = motor(PORT2, ratio18_1, false);
controller Controller1 = controller(primary);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}