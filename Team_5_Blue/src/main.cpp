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
// Controller1          controller                    
// LeftClawMotor        motor         8               
// RightClawMotor       motor         3               
// GyroSensor           inertial      4               
// LeftFrontMotor       motor         10              
// RightFrontMotor      motor         9               
// LeftBackMotor        motor         2               
// RightBackMotor       motor         1               
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
const int TICKS_PER_LOOP_ARM = 2000; // number of ticks to rotate the arm motor in each loop
const int TICKS_PER_LOOP_STACKER = 2000; // number of ticks to rotate the stacker motor in each loop
const int MAX_STACKER_RETRIES_BEFORE_BREAKING_FROM_LOOP = 2; //number of retries before we break from the moveStacker loop
const int MAX_ARMS_RETRIES_BEFORE_BREAKING_FROM_LOOP = 2; //number of retries before we break from the moveArms loop
const double WAIT_FOR_CUBE_INTAKE = 0.2; 
const float WHEEL_DIAMETER = 4.125; // inches
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.1416;
const int TICKS_PER_REVOLUTION_18 = 900; // number of ticks per revolution for 18:1 gear ratio
const int TICKS_PER_LOOP = 450; // number of ticks to rotate the base motors in each loop
const int MAX_BASE_RETRIES_BEFORE_BREAKING_FROM_LOOP = 4; // number of retries before moveRobot breaks from while loop
const int MAX_CLAWS_RETRIES_BEFORE_BREAKING_FROM_LOOP = 2;
const int TICKS_PER_LOOP_CLAWS = 2000; //number of ticks to rotate the claw motors in each loop

/**
**/
double convertInchesIntoTicks_18(float inches) {
  // Convert inches into ticks (based on gear ratio)
  // 900 ticks per revolution for 18:1 gear ratio
  double ticks = TICKS_PER_REVOLUTION_18*inches/WHEEL_CIRCUMFERENCE;
  return ticks;
}


void logTextClearScreen(const std::string& str)
{
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(str.c_str());
}

/**
**/
void logText(const std::string& str)
{
  Controller1.Screen.newLine();
  Controller1.Screen.print(str.c_str());
}

/**
**/
void logText(const std::string& str, double d)
{
  Controller1.Screen.newLine();
  Controller1.Screen.print(str.c_str());
  Controller1.Screen.newLine(); 
  Controller1.Screen.print("%lf", d);
}

/**
**/
void logText(double d1, double d2)
{
  Controller1.Screen.newLine();
  Controller1.Screen.print("%lf", d1);
  Controller1.Screen.newLine(); 
  Controller1.Screen.print("%lf", d2);
}

int convertDegreesIntoTicks_36(float degrees) {
    // Convert degrees into ticks (based on gear ratio)
    // 1,800 ticks per revolution for 36:1 gear ratio
    double ticks = TICKS_PER_REVOLUTION_36*degrees/360;
    return ticks;  
}

void moveStacker(float degrees, int speed, int direction)
{
    // 1. Convert degrees into ticks
    double ticks = convertDegreesIntoTicks_36(degrees);

    // 2. Set the initial motor encoder counters to 0
    StackerMotor.setRotation(0, vex::rotationUnits::raw);
  
    // 3. Set the velocity of the motor to 'speed'
    StackerMotor.setVelocity(speed, velocityUnits::pct);

    // 4. Create counter variable and set it to 0
    double ticksSM = 0;
    double lastTicksSM = 0;
    int stackMotorNotMoved = 0;

    // 5. Loop - while (counter variable < ticks)
    while (ticksSM < ticks) {
      double moveSMTicks = 0;

      if (ticksSM < ticks) {
        // read the counter value
        ticksSM = StackerMotor.rotation(vex::rotationUnits::raw)*direction;

        // calculate remainingTicks to move
        double remainingTicks = ticks - ticksSM;

        // if remainingTicks > TICKS_PER_LOOP_STACKER(), set moveTicks (variable) = TICKS_PER_LOOP_STACKER
        // else set moveTicks = remainingTicks
        if (remainingTicks > TICKS_PER_LOOP_STACKER) {
          moveSMTicks = TICKS_PER_LOOP_STACKER;
        } else {
          moveSMTicks = remainingTicks;
        }

        // rotateFor(moveTicks, raw, false)
        StackerMotor.rotateFor(moveSMTicks*direction, vex::rotationUnits::raw, false);
      }

      /* 
        Motor rotates at 100rpm @ 100% speed
        => 1.667 rps (revolutions per second)
        => 3000 ticks per second (for 1800 ticks/revolution)

        @ 100% speed
        2000 ticks(TICKS_PER_LOOP_STACKER) => 2000/3000 s = 666.667 ms
              
        @ 75% speed
        2000 ticks(TICKS_PER_LOOP_STACKER) => 666.667 ms * 100 / 75 = 888.889 ms

        2000 ticks(TICKS_PER_LOOP_STACKER) => 2000 * 100 / 3 / speed ms = 2000 * 33.33 / speed ms
      */
      wait(moveSMTicks * 33.33 / speed, msec);
      if (ticksSM == lastTicksSM) {
        stackMotorNotMoved++;
        if (stackMotorNotMoved >= MAX_STACKER_RETRIES_BEFORE_BREAKING_FROM_LOOP) {
          // stack motor has not moved for max retries times. Break out of while loop
          break;
        }
      } else {
        // reset the value of stackMotorNotMoved
        stackMotorNotMoved = 0;
        lastTicksSM = ticksSM;
      } 
    }
    // We are done moving the stacker  
}

void setStartingPosition() 
{
  // Raise the arms
  ArmMotor.setVelocity(100, velocityUnits::pct); 
  ArmMotor.rotateFor(400, vex::rotationUnits::deg);

  // Bring the arms back to the starting position
  ArmMotor.rotateFor(-400, vex::rotationUnits::deg);
}

void startSpinningClaws(int direction, int speed)
{
  // 1. Set the speed for both the claws
  LeftClawMotor.setVelocity(speed, vex::velocityUnits::pct);
  RightClawMotor.setVelocity(speed, vex::velocityUnits::pct);

  // 2. Now begin spinning the claws
  if (direction == 1) {
    LeftClawMotor.spin(reverse);
  } else if (direction == -1) {
    LeftClawMotor.spin(forward);
  }

  if (direction == 1) {
    RightClawMotor.spin(reverse);
  } else if (direction == -1) {
    RightClawMotor.spin(forward);
  }

  // We are done starting to spin the claw motors at the 'speed'
}

void moveRobot(float inches, int speed, int direction) 
{
 
    // 1. Convert inches into ticks
    double ticks = convertInchesIntoTicks_18(inches);

    // 2. Set the initial motor encoder counters to 0
    LeftFrontMotor.setRotation(0, vex::rotationUnits::raw);
    LeftBackMotor.setRotation(0, vex::rotationUnits::raw);
    RightFrontMotor.setRotation(0, vex::rotationUnits::raw);
    RightBackMotor.setRotation(0, vex::rotationUnits::raw);
  
    // 3. Set the velocity of the motors to 'speed'
    LeftFrontMotor.setVelocity(speed, velocityUnits::pct);
    RightFrontMotor.setVelocity(speed, velocityUnits::pct);
    LeftBackMotor.setVelocity(speed, velocityUnits::pct);
    RightBackMotor.setVelocity(speed, velocityUnits::pct);

    // 4. Create counter variables and set them to 0
    double ticksLFM = 0;
    double ticksRFM = 0;
    double ticksLBM = 0;
    double ticksRBM = 0;
    double lastTicksRFM = 0;
    int RFMotorNotMoved = 0;
    int LFMotorNotMoved = 0;
    int RBMotorNotMoved = 0;
    int LBMotorNotMoved = 0;
    double lastTicksLFM = 0;
    double lastTicksRBM = 0;
    double lastTicksLBM = 0;
    double moveRFMTicks = 0;
    double moveLFMTicks = 0;
    double moveLBMTicks = 0;
    double moveRBMTicks = 0;

    // 5. Loop - while (any counter variable < ticks)
    while (ticksLFM < ticks || ticksRFM < ticks || 
           ticksLBM < ticks || ticksRBM < ticks) {
      
          if (ticksLFM < ticks) {
            moveLFMTicks = 0;

            // read the counter value
            ticksLFM = LeftFrontMotor.rotation(vex::rotationUnits::raw)*direction;

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksLFM;

            // if remainingTicks > TICKS_PER_LOOP, set moveTicks (variable) = TICKS_PER_LOOP
            // else set moveTicks = remainingTicks
            if (remainingTicks > TICKS_PER_LOOP) {
              moveLFMTicks = TICKS_PER_LOOP;
            } else {
              moveLFMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            LeftFrontMotor.rotateFor(moveLFMTicks*direction, vex::rotationUnits::raw, false);
          }

          if (ticksLBM < ticks) {
            moveLBMTicks = 0;

            // read the counter value
            ticksLBM = LeftBackMotor.rotation(vex::rotationUnits::raw)*direction;

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksLBM;

            // if remainingTicks > TICKS_PER_LOOP, set moveTicks (variable) = TICKS_PER_LOOP
            // else set moveTicks = remainingTicks
            if (remainingTicks > TICKS_PER_LOOP) {
              moveLBMTicks = TICKS_PER_LOOP;
            } else {
              moveLBMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            LeftBackMotor.rotateFor(moveLBMTicks*direction, vex::rotationUnits::raw, false);
          }

          if (ticksRBM < ticks) {
            moveRBMTicks = 0;

            // read the counter value
            ticksRBM = RightBackMotor.rotation(vex::rotationUnits::raw)*direction;

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksRBM;

            // if remainingTicks > TICKS_PER_LOOP, set moveTicks (variable) = TICKS_PER_LOOP
            // else set moveTicks = remainingTicks
            if (remainingTicks > TICKS_PER_LOOP) {
              moveRBMTicks = TICKS_PER_LOOP;
            } else {
              moveRBMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            RightBackMotor.rotateFor(moveRBMTicks*direction, vex::rotationUnits::raw, false);
          }   

          if (ticksRFM < ticks) {
            double moveRFMTicks = 0;

            // read the counter value
            ticksRFM = RightFrontMotor.rotation(vex::rotationUnits::raw)*direction;

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksRFM;

            // if remainingTicks > TICKS_PER_LOOP, set moveTicks (variable) = TICKS_PER_LOOP
            // else set moveTicks = remainingTicks
            if (remainingTicks > TICKS_PER_LOOP) {
              moveRFMTicks = TICKS_PER_LOOP;
            } else {
              moveRFMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            RightFrontMotor.rotateFor(moveRFMTicks*direction, vex::rotationUnits::raw, false);
          }

          /*
              Motor rotates at 200rpm @ 100% speed
              => 3.33 rps (revolutions per second)
              => 3,000 ticks per second (for 900 ticks/revolution)

              @ 100% speed
              1 inch = 69.44 ticks => 0.023146 seconds => 23.146 ms
              TICKS_PER_LOOP will take TICKS_PER_LOOP/3,000 secs => TICKS_PER_LOOP*1000/3000 msec
                             will take TICKS_PER_LOOP*0.333 msec

              @ 75% speed
              1 inch = 69.44 ticks => 0.030862 seconds => 30.862 ms
              TICKS_PER_LOOP will take TICKS_PER_LOOP*0.33333/(75/100) msec 
                             will take TICKS_PER_LOOP*33.333/75 msec
                                       
          */ 
          double maxTicksToMove = 0;
          if (moveRFMTicks > maxTicksToMove) {
            maxTicksToMove = moveRFMTicks;
          }
          if (moveLFMTicks > maxTicksToMove) {
            maxTicksToMove = moveLFMTicks;
          }
          if (moveRBMTicks > maxTicksToMove) {
            maxTicksToMove = moveRBMTicks;
          }
          if (moveLBMTicks > maxTicksToMove) {
            maxTicksToMove = moveLBMTicks;
          }
          wait(maxTicksToMove*33.333/speed, msec);

          if (ticksRFM == lastTicksRFM) {
            RFMotorNotMoved++;
            if (RFMotorNotMoved >= MAX_BASE_RETRIES_BEFORE_BREAKING_FROM_LOOP) {
              // RightFrontMotor has not moved for max retries times. Break out of while loop
              return;
            }
          } else {
            // reset the value of RightFrontMotorNotMoved
            RFMotorNotMoved = 0;
            lastTicksRFM = ticksRFM;
          } 
          if (ticksLFM == lastTicksLFM) {
            LFMotorNotMoved++;
            if (LFMotorNotMoved >= MAX_BASE_RETRIES_BEFORE_BREAKING_FROM_LOOP) {
              // LeftFrontMotor has not moved for max retries times. Break out of while loop
              break;
            }
          } else {
            // reset the value of LeftFrontMotorNotMoved
            LFMotorNotMoved = 0;
            lastTicksLFM = ticksLFM;
          } 
          if (ticksRBM == lastTicksRBM) {
            RBMotorNotMoved++;
            if (RBMotorNotMoved >= MAX_BASE_RETRIES_BEFORE_BREAKING_FROM_LOOP) {
              // RightBackMotor has not moved for max retries times. Break out of while loop
              break;
            }
          } else {
            // reset the value of RightBackMotorNotMoved
            RBMotorNotMoved = 0;
            lastTicksRBM = ticksRBM;
          } 
          if (ticksLBM == lastTicksLBM) {
            LBMotorNotMoved++;
            if (LBMotorNotMoved >= MAX_BASE_RETRIES_BEFORE_BREAKING_FROM_LOOP) {
              // LeftBackMotor has not moved for max retries times. Break out of while loop
              break;
            }
          } else {
            // reset the value of LeftBackMotorNotMoved
            LBMotorNotMoved = 0;
            lastTicksLBM = ticksLBM;
          } 
    }
    // We are done moving the robot
}

void stopSpinningClaws()
{
  // Stop spinning the claws
    LeftClawMotor.stop();
    RightClawMotor.stop();

  // We are done stopping to spin the claw motors
}

void turnRobot(double deg, int speed, int direction)
 {
  // Initializing Robot Configuration. DO NOT REMOVE!
  LeftBackMotor.setVelocity(speed,percent);
  LeftFrontMotor.setVelocity(speed,percent);
  RightFrontMotor.setVelocity(speed,percent);
  RightBackMotor.setVelocity(speed,percent);
  // Get the current gyro sensor value

  double currentDeg = GyroSensor.rotation(degrees);

  // Add the 'deg' to the current value as the new 'deg' value to turn
  deg = deg - currentDeg; 


  if (direction > 0){
    LeftBackMotor.spin(forward);
    LeftFrontMotor.spin(forward);
    RightFrontMotor.spin(reverse);
    RightBackMotor.spin(reverse);
  }
  else {
    LeftBackMotor.spin(reverse);
    LeftFrontMotor.spin(reverse);
    RightFrontMotor.spin(forward); 
    RightBackMotor.spin(forward);
  }

  // Waits until the motor reaches a 90 degree turn and stops the Left and
  // Right Motors.
  if (direction > 0) {
    waitUntil((GyroSensor.rotation(degrees) >= deg));
  } else {
    waitUntil((GyroSensor.rotation(degrees) <= deg*-1));
  }

  LeftFrontMotor.stop();
  RightFrontMotor.stop();
  LeftBackMotor.stop();
  RightBackMotor.stop();
}

void moveClaws(float degrees, int speed, int direction) 
{
    // 1. Convert inches into ticks
    double ticks = convertDegreesIntoTicks_36(degrees);

    // 2. Set the initial motor encoder counters to 0
    LeftClawMotor.setRotation(0, vex::rotationUnits::raw);
    RightClawMotor.setRotation(0, vex::rotationUnits::raw);
  
    // 3. Set the velocity of the motors to 'speed'
    LeftClawMotor.setVelocity(speed, velocityUnits::pct);
    RightClawMotor.setVelocity(speed, velocityUnits::pct);

    // 4. Create counter variables and set them to 0
    double ticksLCM = 0;
    double ticksRCM = 0;
    double lastTicksLCM = 0;
    double lastTicksRCM = 0;
    int LCMotorNotMoved = 0;
    int RCMotorNotMoved = 0;

    // 5. Loop - while (any counter variable < ticks)
    double moveLCMTicks = 0;
    double moveRCMTicks = 0;

    while (ticksLCM < ticks || ticksRCM < ticks) { 
      if (ticksLCM < ticks) {
        moveLCMTicks = 0;

        // read the counter value
        ticksLCM = LeftClawMotor.rotation(vex::rotationUnits::raw)*direction;

        // calculate remainingTicks to move
        double remainingTicks = ticks - ticksLCM;

        // if remainingTicks > TICKS_PER_LOOP_CLAWS, set moveTicks (variable) = TICKS_PER_LOOP_CLAWS
        // else set moveTicks = remainingTicks
        if (remainingTicks > TICKS_PER_LOOP_CLAWS) {
          moveLCMTicks = TICKS_PER_LOOP_CLAWS;
        } else {
          moveLCMTicks = remainingTicks;
        }

        // rotateFor(moveTicks, raw, false)
        LeftClawMotor.rotateFor(moveLCMTicks*direction, vex::rotationUnits::raw, false);
      }

      if (ticksRCM < ticks) {
        moveRCMTicks = 0;

        // read the counter value
        ticksRCM = RightClawMotor.rotation(vex::rotationUnits::raw)*direction;

        // calculate remainingTicks to move
        double remainingTicks = ticks - ticksRCM;

        // if remainingTicks > TICKS_PER_LOOP_CLAWS, set moveTicks (variable) = TICKS_PER_LOOP_CLAWS
        // else set moveTicks = remainingTicks
        if (remainingTicks > TICKS_PER_LOOP_CLAWS) {
          moveRCMTicks = TICKS_PER_LOOP_CLAWS;
        } else {
          moveRCMTicks = remainingTicks;
        }

        // rotateFor(moveTicks, raw, false)
        RightClawMotor.rotateFor(moveRCMTicks*direction, vex::rotationUnits::raw, false);
      }   

      /*
        Motor rotates at 100rpm @ 100% speed (36:1)
        => 1.6667 rps (revolutions per second)
        => 3,000 ticks per second (for 1,800 ticks/revolution)

        @ 100% speed
        TICKS_PER_LOOP will take TICKS_PER_LOOP/3,000 secs => TICKS_PER_LOOP*1000/3000 msec
                       will take TICKS_PER_LOOP*0.333 msec

        @ 75% speed
        TICKS_PER_LOOP will take TICKS_PER_LOOP*0.333/(75/100) msec 
                       will take TICKS_PER_LOOP*33.33/75 msec                                 
      */ 

      if (moveRCMTicks > moveLCMTicks) {
        wait(moveRCMTicks*33.33/speed, msec);
      } else {
        wait(moveLCMTicks*33.33/speed, msec);
      }

      if (ticksLCM == lastTicksLCM) {
        LCMotorNotMoved++;
        if (LCMotorNotMoved >= MAX_CLAWS_RETRIES_BEFORE_BREAKING_FROM_LOOP) {
          // LeftClawMotor has not moved for max retries times. Break out of while loop
          break;
        }
      } else {
        // reset the value of LCMotorNotMoved
        LCMotorNotMoved = 0;
        lastTicksLCM = ticksLCM;
      } 

      if (ticksRCM == lastTicksRCM) {
        RCMotorNotMoved++;
        if (RCMotorNotMoved >= MAX_CLAWS_RETRIES_BEFORE_BREAKING_FROM_LOOP) {
          // RightClawMotor has not moved for max retries times. Break out of while loop
          break;
        }
      } else {
        // reset the value of LCMotorNotMoved
        RCMotorNotMoved = 0;
        lastTicksRCM = ticksRCM;
      }
    }
    // We are done moving the claws
}

void moveArms(float degrees, int speed, int direction)
{
    // 1. Convert degrees into ticks
    double ticks = convertDegreesIntoTicks_36(degrees);

    // 2. Set the initial motor encoder counter to 0
    ArmMotor.setRotation(0, vex::rotationUnits::raw);
  
    // 3. Set the velocity of the motor to 'speed'
    ArmMotor.setVelocity(speed, velocityUnits::pct);

    // 4. Create counter variable and set it to 0
    double ticksAM = 0;
    double lastTicksAM = 0;
    int armMotorNotMoved = 0;

    // 5. Loop - while (counter variable < ticks)
    while (ticksAM < ticks) { 
      double moveAMTicks = 0;

      // read the counter value
      ticksAM = ArmMotor.rotation(vex::rotationUnits::raw)*direction;

      // calculate remainingTicks to move
      double remainingTicks = ticks - ticksAM;

      // if remainingTicks > TICKS_PER_LOOP_ARM(2000), set moveTicks (variable) = TICKS_PER_LOOP_ARM
      // else set moveTicks = remainingTicks
      if (remainingTicks > TICKS_PER_LOOP_ARM) {
        moveAMTicks = TICKS_PER_LOOP_ARM;
      } else {
        moveAMTicks = remainingTicks;
      }

      ArmMotor.rotateFor(moveAMTicks, vex::rotationUnits::raw, false);

      /* 
        Motor rotates at 100rpm @ 100% speed
        => 1.667 rps (revolutions per second)
        => 3000 ticks per second (for 1800 ticks/revolution)

        @ 100% speed
        2000 ticks(TICKS_PER_LOOP_ARM) => 2000/3000 s = 666.667 ms
        
        @ 75% speed
        2000 ticks(TICKS_PER_LOOP_ARM) => 666.667 ms * 100 / 75 = 888.889 ms

        2000 ticks(TICKS_PER_LOOP_ARM) => 666.667 * 100 / speed ms = 2000 * 33.33 / speed ms
      */
      wait(moveAMTicks * 33.33 / speed, msec);

      // check if the motor has moved. If not, break out of the loop after max retries
      if (ticksAM == lastTicksAM) {
        armMotorNotMoved++;
        if (armMotorNotMoved >= MAX_ARMS_RETRIES_BEFORE_BREAKING_FROM_LOOP) {
          // arm motor has not moved for max retries times. Break out of while loop
          break;
        }
      } else {
        // reset the value of armMotorNotMoved
        armMotorNotMoved = 0;
        lastTicksAM = ticksAM;
      } 
    }
    
    // We are done moving the arms
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  GyroSensor.calibrate();
  while (GyroSensor.isCalibrating()) {
    wait(100, msec);
  }
  Controller1.Screen.print("GyroSensor Calibrated");   

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
  setStartingPosition();
  startSpinningClaws(-1, 100);

  // Pick up the pre-load
  //wait(WAIT_FOR_CUBE_INTAKE, sec);

  // waits for the Inertial Sensor to calibrate
  /*while (GyroSensor.isCalibrating()) {
    wait(100, msec);
  }*/
  
  moveRobot(12,50,1);
  moveRobot(6,25,1); 
  
  // Pick up cube #2
  wait(WAIT_FOR_CUBE_INTAKE, sec);
  for (int i = 0; i < 2; i++){
    moveRobot(6,25,1);
    // Pick up cube #3 & #4
    wait(WAIT_FOR_CUBE_INTAKE,sec);
  }
  wait(600, msec);
  moveRobot(9,25,1);

  // Pick up cube #5
  wait(450, msec);
  stopSpinningClaws();

  // Move backward to goal
  moveRobot(22,75,-1);
  turnRobot(105,15,-1);
  moveRobot(14.5,50,1);

  moveStacker(500, 50, 1);
  //moveClaws(60,50,-1);
  moveStacker(140, 20, 1);

  moveRobot(8,25,-1);

  // ..........................................................................

  // An instance of brain used for printing to the V5 Brain screen
  brain  Brain;

  // VEXcode device constructors
  motor RightFrontBaseMotor = motor(PORT9, ratio18_1, false);
  motor LeftFrontBaseMotor = motor(PORT10, ratio18_1, false);
  motor LeftDriveSmart = motor(PORT2, ratio18_1, false);
  motor RightDriveSmart = motor(PORT1, ratio18_1, true);
 
  //drivetrain Drivetrain = drivetrain(LeftFrontBaseMotor, RightFrontBaseMotor, 319.19, 295, 130, mm, 1);

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
  motor LeftBackDriveSmart = motor(PORT2, ratio18_1, false);
  motor LeftFrontDriveSmart = motor(PORT10, ratio18_1, false);
  drivetrain LeftDrivetrain = drivetrain(LeftBackDriveSmart, LeftFrontDriveSmart, 319.19, 295, 130, mm, 1);

  motor RightBackDriveSmart = motor(PORT1, ratio18_1, true);
  motor RightFrontDriveSmart = motor(PORT9, ratio18_1, true);
  drivetrain RightDrivetrain = drivetrain(RightBackDriveSmart, RightFrontDriveSmart, 319.19, 295, 130, mm, 1);

  // VEXcode generated functions
  // define variables used for controlling motors based on controller inputs
  //bool Controller1LeftShoulderControlMotorsStopped = true;
  //bool Controller1RightShoulderControlMotorsStopped = true;
  bool Controller1L1ButtonMotorsStopped = true;
  bool Controller1R1ButtonMotorsStopped = true;
  bool Controller1L2ButtonMotorsStopped = true;
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

    if (Controller1.ButtonL1.pressing()) {
      RightClawMotor.setVelocity(100, pct);
      RightClawMotor.spin(forward);

      LeftClawMotor.setVelocity(100, pct);
      LeftClawMotor.spin(forward);
              
      Controller1L1ButtonMotorsStopped = false;
    } else if (!Controller1L1ButtonMotorsStopped) {
       RightClawMotor.stop();
       LeftClawMotor.stop();
       
       // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
       Controller1L1ButtonMotorsStopped = true;
    }
    
    if (Controller1.ButtonR1.pressing()) {
      RightClawMotor.setVelocity(100, pct);
      RightClawMotor.spin(reverse);

      LeftClawMotor.setVelocity(100, pct);
      LeftClawMotor.spin(reverse);
          
      Controller1R1ButtonMotorsStopped = false;
    } else if (!Controller1R1ButtonMotorsStopped) {
      RightClawMotor.stop();
      LeftClawMotor.stop();
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1R1ButtonMotorsStopped = true;
    }

    if (Controller1.ButtonL2.pressing()) {
      LeftFrontDriveSmart.setVelocity(25, pct);
      RightFrontDriveSmart.setVelocity(25, pct);
      RightBackDriveSmart.setVelocity(25, pct);
      LeftBackDriveSmart.setVelocity(25, pct);
      LeftFrontDriveSmart.spin(forward);
      RightFrontDriveSmart.spin(forward);
      RightBackDriveSmart.spin(forward);
      LeftBackDriveSmart.spin(forward);
  
      Controller1L2ButtonMotorsStopped = false;

    } else if (!Controller1L2ButtonMotorsStopped) {
      LeftFrontDriveSmart.stop();
      RightFrontDriveSmart.stop();
      RightBackDriveSmart.stop();
      LeftBackDriveSmart.stop();
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1L2ButtonMotorsStopped = true;
    }
/*
    if (Controller1.ButtonR2.pressing()) {
      LeftFrontDriveSmart.setVelocity(100, pct);
      RightFrontDriveSmart.setVelocity(100, pct);
      RightBackDriveSmart.setVelocity(100, pct);
      LeftBackDriveSmart.setVelocity(100, pct);
      LeftFrontDriveSmart.spin(reverse);
      RightFrontDriveSmart.spin(reverse);
      RightBackDriveSmart.spin(reverse);
      LeftBackDriveSmart.spin(reverse);
  
      Controller1R2ButtonMotorsStopped = false;

    } else if (!Controller1R2ButtonMotorsStopped) {
      LeftFrontDriveSmart.stop();
      RightFrontDriveSmart.stop();
      RightBackDriveSmart.stop();
      LeftBackDriveSmart.stop();
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1R2ButtonMotorsStopped = true;
    }    
    */
    if (Controller1.ButtonY.pressing()) {
        moveStacker(500, 50, 1);
        moveStacker(125, 20, 1);
        ControllerYButton = false;
      } else if (!ControllerYButton){
        StackerMotor.stop();
        StackerMotor.setBrake(hold);
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        ControllerYButton = true;
      }
    
    if (Controller1.ButtonA.pressing()) {
        moveStacker(625, 50, -1);
        ControllerAButton = false;
      } else if (!ControllerAButton){
        StackerMotor.stop();
        StackerMotor.setBrake(hold);
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        ControllerAButton = true;
      }
    
    if (Controller1.ButtonLeft.pressing()) {
        moveArms(450, 50, 1);
        ControllerLeftButton = false;
      } else if (!ControllerLeftButton){
        ArmMotor.stop();
        ArmMotor.setBrake(hold);
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        ControllerLeftButton = true;
      }

    if (Controller1.ButtonRight.pressing()) {
        moveStacker(200, 50, 1);
        moveArms(675, 50, 1);
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
    // Sleep the task for a short amount of time to
    // prevent wasted resources.
    wait(20, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {

  // Run the pre-autonomous function.
  pre_auton();
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
