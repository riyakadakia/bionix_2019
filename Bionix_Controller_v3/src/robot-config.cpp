#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftBackDriveSmart = motor(PORT2, ratio18_1, false);
motor LeftFrontDriveSmart = motor(PORT10, ratio18_1, false);
drivetrain LeftDrivetrain = drivetrain(LeftBackDriveSmart, LeftFrontDriveSmart, 319.19, 295, 130, mm, 1);

motor RightBackDriveSmart = motor(PORT1, ratio18_1, true);
motor RightFrontDriveSmart = motor(PORT9, ratio18_1, true);
drivetrain RightDrivetrain = drivetrain(RightBackDriveSmart, RightFrontDriveSmart, 319.19, 295, 130, mm, 1);

motor ArmMotor = motor(PORT6, ratio36_1, true);
motor StackerMotor = motor(PORT7, ratio36_1, false);

motor LeftClawMotor = motor(PORT8, ratio18_1, false);
motor RightClawMotor = motor(PORT3, ratio18_1, true);

// VEXcode generated functions
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool Controller1UpDownButtonsControlMotorsStopped = true;
bool Controller1XBButtonsControlMotorsStopped = true;
bool DrivetrainLeftNeedsToBeStopped_Controller1 = true;
bool DrivetrainRightNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_callback_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    // calculate the drivetrain motor velocities from the controller joystick axies
    // left = Axis3 + Axis4
    // right = Axis3 - Axis4
    int drivetrainLeftAddSideSpeed = Controller1.Axis3.position() + Controller1.Axis4.position();
    int drivetrainLeftSubSideSpeed = Controller1.Axis3.position() - Controller1.Axis4.position();

    int drivetrainRightAddSideSpeed = Controller1.Axis2.position() + Controller1.Axis1.position();
    int drivetrainRightSubSideSpeed = Controller1.Axis2.position() - Controller1.Axis1.position();

    // check if the values are inside of the deadband range
    if (abs(drivetrainLeftAddSideSpeed) < 5 && abs(drivetrainLeftSubSideSpeed) < 5) {
      // check if the motors have already been stopped
      if (DrivetrainLeftNeedsToBeStopped_Controller1) {
        // stop the drive motors
        LeftBackDriveSmart.stop();
        LeftFrontDriveSmart.stop();

        // tell the code that the motors have been stopped
        DrivetrainLeftNeedsToBeStopped_Controller1 = false;
      }
    } else {
      // reset the toggle so that the deadband code knows to stop the motors next time the input is in the deadband range
      DrivetrainLeftNeedsToBeStopped_Controller1 = true;
    }

    // check if the values are inside of the deadband range
    if (abs(drivetrainRightAddSideSpeed) < 5 && abs(drivetrainRightSubSideSpeed) < 5) {
      // check if the motors have already been stopped
      if (DrivetrainRightNeedsToBeStopped_Controller1) {
        // stop the drive motors
        RightBackDriveSmart.stop();
        RightFrontDriveSmart.stop();

        // tell the code that the motors have been stopped
        DrivetrainRightNeedsToBeStopped_Controller1 = false;
      }
    } else {
      // reset the toggle so that the deadband code knows to stop the motors next time the input is in the deadband range
      DrivetrainRightNeedsToBeStopped_Controller1 = true;
    }

    // only tell the left drive motor to spin if the values are not in the deadband range
    if (DrivetrainLeftNeedsToBeStopped_Controller1) {
      LeftBackDriveSmart.setVelocity(drivetrainLeftAddSideSpeed, percent);
      LeftBackDriveSmart.spin(forward);
      LeftFrontDriveSmart.setVelocity(drivetrainLeftSubSideSpeed, percent);
      LeftFrontDriveSmart.spin(forward);
    }

    // only tell the right drive motor to spin if the values are not in the deadband range

    if (DrivetrainRightNeedsToBeStopped_Controller1) {
      RightBackDriveSmart.setVelocity(drivetrainRightAddSideSpeed, percent);
      RightBackDriveSmart.spin(forward);         
      RightFrontDriveSmart.setVelocity(drivetrainRightSubSideSpeed, percent);
      RightFrontDriveSmart.spin(forward);
    }

    // check the ButtonR1/ButtonR2 status to control LeftClawMotor
    if (Controller1.ButtonR1.pressing()) {
      RightClawMotor.setVelocity(100, pct);
      RightClawMotor.spin(forward);
      Controller1RightShoulderControlMotorsStopped = false;
    } else if (Controller1.ButtonR2.pressing()) {
      RightClawMotor.setVelocity(100, pct);
      RightClawMotor.spin(reverse);
      Controller1RightShoulderControlMotorsStopped = false;
    } else if (!Controller1RightShoulderControlMotorsStopped) {
      RightClawMotor.stop();
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1RightShoulderControlMotorsStopped = true;
    }

    if (Controller1.ButtonL1.pressing()) {
      LeftClawMotor.setVelocity(100, pct);
      LeftClawMotor.spin(forward);
      Controller1LeftShoulderControlMotorsStopped = false;
    } else if (Controller1.ButtonL2.pressing()) {
      LeftClawMotor.setVelocity(100, pct);
      LeftClawMotor.spin(reverse);
      Controller1LeftShoulderControlMotorsStopped = false;
    } else if (!Controller1LeftShoulderControlMotorsStopped) {
      LeftClawMotor.stop();
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1LeftShoulderControlMotorsStopped = true;
    }

    // check the Up/Down Buttons status to control ArmMotor
    if (Controller1.ButtonUp.pressing()) {
      ArmMotor.spin(forward);
      Controller1UpDownButtonsControlMotorsStopped = false;
    } else if (Controller1.ButtonDown.pressing()) {
      ArmMotor.spin(reverse);
      Controller1UpDownButtonsControlMotorsStopped = false;
    } else if (!Controller1UpDownButtonsControlMotorsStopped){
      ArmMotor.stop();
      ArmMotor.setBrake(hold);
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1UpDownButtonsControlMotorsStopped = true;
    }

    // check the X/B buttons status to control StackerMotor
    if (Controller1.ButtonX.pressing()) {
      StackerMotor.spin(forward);
      Controller1XBButtonsControlMotorsStopped = false;
    } else if (Controller1.ButtonB.pressing()) {
      StackerMotor.spin(reverse);
      Controller1XBButtonsControlMotorsStopped = false;
    } else if (!Controller1XBButtonsControlMotorsStopped){
      StackerMotor.stop();
      StackerMotor.setBrake(hold);
      Controller1XBButtonsControlMotorsStopped = true;
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_callback_Controller1);
}