using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor RightClawMotor;
extern drivetrain Drivetrain;
extern motor ArmMotor;
extern motor StackerMotor;
extern motor LeftClawMotor;
extern motor RightFrontBaseMotor;
extern motor LeftFrontBaseMotor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );