/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ArmMotor             motor         6               
// StackerMotor         motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
const int TICKS_PER_REVOLUTION_36 = 1800; // number of ticks per revolution for 36:1 gear ratio
const int TICKS_PER_LOOP_ARM = 2000; //number of ticks to rotate the arm motor in each loop
const int TICKS_PER_LOOP_STACKER = 2000; // number of ticks to rotate the stacker motor in each loop
const int TICKS_PER_LOOP_STACKER_DOWN = -2000;

int convertDegreesIntoTicks_36(float degrees) {
    // Convert degrees into ticks (based on gear ratio)
    // 1,800 ticks per revolution for 36:1 gear ratio
    double ticks = TICKS_PER_REVOLUTION_36*degrees/360;
    return ticks;  
}


void moveStackerUp(float degrees, int speed)
{
    // 1. Convert degrees into ticks
    double ticks = convertDegreesIntoTicks_36(degrees);

    // 2. Set the initial motor encoder counters to 0
    StackerMotor.setRotation(0, vex::rotationUnits::raw);
  
    // 3. Set the velocity of the motor to 'speed'
    StackerMotor.setVelocity(speed, velocityUnits::pct);

    // 4. Create counter variable and set it to 0
    double ticksSM = 0;

    // 5. Loop - while (counter variable < ticks)
    while (ticksSM < ticks) {
      
          if (ticksSM < ticks) {
            double moveSMTicks = 0;

            // read the counter value
            ticksSM = StackerMotor.rotation(vex::rotationUnits::raw);

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksSM;

            // if remainingTicks > TICKS_PER_LOOP_STACKER(50), set moveTicks (variable) = TICKS_PER_LOOP_STACKER
            // else set moveTicks = remainingTicks
            if (remainingTicks > TICKS_PER_LOOP_STACKER) {
              moveSMTicks = TICKS_PER_LOOP_STACKER;
            } else {
              moveSMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            StackerMotor.rotateFor(moveSMTicks, vex::rotationUnits::raw, false);
          }
          /* 
              Motor rotates at 200rpm @ 100% speed
              => 3.33 rps (revolutions per second)
              => 6,000 ticks per second (for 1800 ticks/revolution)

              @ 100% speed
              50 ticks(TICKS_PER_LOOP_STACKER) => 50/6000 s = 8.333 ms
              
              @ 75% speed
              50 ticks(TICKS_PER_LOOP_STACKER) => 8.333 ms * 100 / 75 = 11.108 ms

              50 ticks(TICKS_PER_LOOP_STACKER) => 50 * 100 / 60 /speed ms = 50 * 1.667 / speed ms
          */
          wait(TICKS_PER_LOOP_STACKER*1.667/speed, msec);
    }
    // We are done moving the stacker  
}

void moveStackerDown(float degrees, int speed)
{
    // 1. Convert degrees into ticks
    double ticks = convertDegreesIntoTicks_36(degrees);

    // 2. Set the initial motor encoder counters to 0
    StackerMotor.setRotation(0, vex::rotationUnits::raw);
  
    // 3. Set the velocity of the motor to 'speed'
    StackerMotor.setVelocity(speed, velocityUnits::pct);
   
    // 4. Create counter variable and set it to 0
    double ticksSM = 0;

    // 5. Loop - while (counter variable < ticks)
    while (ticksSM > ticks) {
      
          if (ticksSM > ticks) {
            double moveSMTicks = 0;

            // read the counter value
          
            ticksSM = StackerMotor.rotation(vex::rotationUnits::raw);

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksSM;

            // if remainingTicks > TICKS_PER_LOOP_STACKER(50), set moveTicks (variable) = TICKS_PER_LOOP_STACKER
            // else set moveTicks = remainingTicks
            if (remainingTicks < TICKS_PER_LOOP_STACKER_DOWN) {
              moveSMTicks = TICKS_PER_LOOP_STACKER_DOWN;
            } else {
              moveSMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            
            StackerMotor.rotateFor(moveSMTicks, vex::rotationUnits::raw, false);
          }
          /* 
              Motor rotates at 200rpm @ 100% speed
              => 3.33 rps (revolutions per second)
              => 6,000 ticks per second (for 1800 ticks/revolution)

              @ 100% speed
              50 ticks(TICKS_PER_LOOP_STACKER) => 50/6000 s = 8.333 ms
              
              @ 75% speed
              50 ticks(TICKS_PER_LOOP_STACKER) => 8.333 ms * 100 / 75 = 11.108 ms

              50 ticks(TICKS_PER_LOOP_STACKER) => 50 * 100 / 60 /speed ms = 50 * 1.667 / speed ms
          */
          wait(TICKS_PER_LOOP_STACKER_DOWN*1.667/speed, msec);
    }
    // We are done moving the stacker  
}


void moveArms(float degrees, int speed)
{
    // 1. Convert degrees into ticks
    double ticks = convertDegreesIntoTicks_36(degrees);
    double ticksAM = 0;

    // 2. Set the initial motor encoder counter to 0
    ArmMotor.setRotation(0, vex::rotationUnits::raw);
  
    // 3. Set the velocity of the motor to 'speed'
    ArmMotor.setVelocity(speed, velocityUnits::pct);

    // 4. Create counter variable and set it to 0
    

    // 5. Loop - while (counter variable < ticks)
    while (ticksAM < ticks) {
      
          if (ticksAM < ticks && !ArmMotor.isSpinning()) {
            double moveAMTicks = 0;

            // read the counter value
            ticksAM = ArmMotor.rotation(vex::rotationUnits::raw);

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksAM;

            // if remainingTicks > TICKS_PER_LOOP_ARM(50), set moveTicks (variable) = TICKS_PER_LOOP_ARM
            // else set moveTicks = remainingTicks
            if (remainingTicks > TICKS_PER_LOOP_ARM) {
              moveAMTicks = TICKS_PER_LOOP_ARM;
            } else {
              moveAMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            ArmMotor.rotateFor(moveAMTicks, vex::rotationUnits::raw, false);
          }
          /* 
              Motor rotates at 200rpm @ 100% speed
              => 3.33 rps (revolutions per second)
              => 6,000 ticks per second (for 1800 ticks/revolution)

              @ 100% speed
              50 ticks(TICKS_PER_LOOP_ARM) => 50/6000 s = 8.333 ms
              
              @ 75% speed
              50 ticks(TICKS_PER_LOOP_ARM) => 8.333 ms * 100 / 75 = 11.108 ms

              50 ticks(TICKS_PER_LOOP_ARM) => 50 * 100 / 60 /speed ms = 50 * 1.667 / speed ms
          */
          wait(TICKS_PER_LOOP_ARM*1.667/speed, msec);
    }
    // We are done moving the arms  
}





void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  // An instance of brain used for printing to the V5 Brain screen
  brain  Brain;

  // VEXcode device constructors
  controller Controller1 = controller(primary);
  motor RightFrontBaseMotor = motor(PORT9, ratio18_1, false);
  motor LeftFrontBaseMotor = motor(PORT10, ratio18_1, false);
  motor RightClawMotor = motor(PORT3, ratio18_1, false);
  motor LeftDriveSmart = motor(PORT2, ratio18_1, false);
  motor RightDriveSmart = motor(PORT1, ratio18_1, true);
  //drivetrain Drivetrain = drivetrain(LeftFrontBaseMotor, RightFrontBaseMotor, 319.19, 295, 130, mm, 1);
  motor ArmMotor = motor(PORT6, ratio36_1, true);
  motor StackerMotor = motor(PORT7, ratio36_1, false);
  motor LeftClawMotor = motor(PORT8, ratio18_1, true);
 
  // Drivetrain.driveFor(forward 1100 ticks
  //Drivetrain.driveFor(reverse 1100 ticks

  // 0. 1 point Auton 
  RightFrontBaseMotor.rotateFor(1100,vex::rotationUnits::raw,false);
  LeftFrontBaseMotor.rotateFor(-1100,vex::rotationUnits::raw,true);

  wait(1, timeUnits::sec);
 
  RightFrontBaseMotor.rotateFor(-1100,vex::rotationUnits::raw,false);
  LeftFrontBaseMotor.rotateFor(1100,vex::rotationUnits::raw,true);

  wait(1, timeUnits::sec);

  // 1. Staring position open
  StackerMotor.rotateFor(200, vex::rotationUnits::deg, false);

  // 2. Raise the arms to open first part of stacker
  ArmMotor.setVelocity(100, velocityUnits::pct); 
  ArmMotor.rotateFor(420, vex::rotationUnits::deg);

  // 3. Spin the claws outward to untangle the stacker if it gets stuck
  RightClawMotor.rotateFor(180, vex::rotationUnits::deg, false);
  LeftClawMotor.rotateFor(180, vex::rotationUnits::deg, true);

  // 4. Bring the arms back to the starting position
  ArmMotor.rotateFor(-400, vex::rotationUnits::deg);

  // 5. Bring the stacker back to the starting position
  StackerMotor.rotateFor(-200, vex::rotationUnits::deg, false);
}

  


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // An instance of brain used for printing to the V5 Brain screen
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
  bool ControllerLeftButton = true;
  bool ControllerRightButton = true;
  bool ControllerAButton = true;
  bool ControllerYButton = true;
  bool DrivetrainLeftNeedsToBeStopped_Controller1 = true;
  bool DrivetrainRightNeedsToBeStopped_Controller1 = true;
  

  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    // calculate the drivetrain motor velocities from the controller joystick axies
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

// Ignore this part i was just testing something
    /*if (Controller1.ButtonR1.pressing()) {
          RightClawMotor.setVelocity(100, pct);
          RightClawMotor.spin(forward);

          LeftClawMotor.setVelocity(100, pct);
          LeftClawMotor.spin(forward);
          
          Controller1LeftShoulderControlMotorsStopped = false;

          Controller1RightShoulderControlMotorsStopped = false;

        } else if (!Controller1RightShoulderControlMotorsStopped) {
          RightClawMotor.stop();
          LeftClawMotor.stop();
          // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
          Controller1RightShoulderControlMotorsStopped = true;
        }
*/

    if (Controller1.ButtonL1.pressing()) {
              RightClawMotor.setVelocity(100, pct);
              RightClawMotor.spin(forward);

              LeftClawMotor.setVelocity(100, pct);
              LeftClawMotor.spin(forward);
              
              Controller1LeftShoulderControlMotorsStopped = false;

              Controller1RightShoulderControlMotorsStopped = false;

            } else if (!Controller1RightShoulderControlMotorsStopped) {
              RightClawMotor.stop();
              LeftClawMotor.stop();
              // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
              Controller1RightShoulderControlMotorsStopped = true;
            }
    
    if (Controller1.ButtonR1.pressing()) {
          RightClawMotor.setVelocity(100, pct);
          RightClawMotor.spin(reverse);

          LeftClawMotor.setVelocity(100, pct);
          LeftClawMotor.spin(reverse);
          
          Controller1LeftShoulderControlMotorsStopped = false;

          Controller1RightShoulderControlMotorsStopped = false;

        } else if (!Controller1RightShoulderControlMotorsStopped) {
          RightClawMotor.stop();
          LeftClawMotor.stop();
          // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
          Controller1RightShoulderControlMotorsStopped = true;
        }
    

       
    
    if (Controller1.ButtonY.pressing()) {
        moveStackerUp(650, 25);
        ControllerYButton = false;
      } else if (!ControllerYButton){
        StackerMotor.stop();
        StackerMotor.setBrake(hold);
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        ControllerYButton = true;
      }

    if (Controller1.ButtonA.pressing()) {
        moveStackerDown(-650, 100);
        ControllerAButton = false;
      } else if (!ControllerAButton){
        StackerMotor.stop();
        StackerMotor.setBrake(hold);
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        ControllerAButton = true;
      }
    
    if (Controller1.ButtonLeft.pressing()) {
        moveArms(400, 50);
        ControllerLeftButton = false;
      } else if (!ControllerLeftButton){
        ArmMotor.stop();
        ArmMotor.setBrake(hold);
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        ControllerLeftButton = true;
      }

    if (Controller1.ButtonRight.pressing()) {
        moveArms(540, 50);
        ControllerRightButton = false;
      } else if (!ControllerRightButton){
        ArmMotor.stop();
        ArmMotor.setBrake(hold);
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        ControllerRightButton = true;
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
      StackerMotor.setVelocity(25, percentUnits::pct);
      StackerMotor.spin(forward);
      Controller1XBButtonsControlMotorsStopped = false;
    } else if (Controller1.ButtonB.pressing()) {
      StackerMotor.setVelocity(25, percentUnits::pct);
      StackerMotor.spin(reverse);
      Controller1XBButtonsControlMotorsStopped = false;
    } else if (!Controller1XBButtonsControlMotorsStopped){
      StackerMotor.stop();
      StackerMotor.setBrake(hold);
      Controller1XBButtonsControlMotorsStopped = true;
    }
    // wait before repeating the process
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
