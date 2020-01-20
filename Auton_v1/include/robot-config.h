using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor RightClawMotor;
extern motor ArmMotor;
extern motor StackerMotor;
extern motor LeftClawMotor;
extern motor RightFrontMotor;
extern motor LeftFrontMotor;
extern motor RightBackMotor;
extern motor LeftBackMotor;
extern controller Controller1;
extern inertial GyroSensor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );