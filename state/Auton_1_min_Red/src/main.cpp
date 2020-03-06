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
// LeftClawMotor        motor         4               
// RightClawMotor       motor         3               
// GyroSensor           inertial      15              
// LeftFrontMotor       motor         10              
// RightFrontMotor      motor         9               
// LeftBackMotor        motor         8               
// RightBackMotor       motor         1               
// BumperA              bumper        A               
// BumperH              bumper        H               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

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
const int TICKS_BUFFER = 10; // number of ticks that the motor may not move
const int MAX_ARMS_RETRIES_BEFORE_BREAKING_FROM_LOOP = 2; //number of retries before we break from the moveArms loop
const double WAIT_FOR_CUBE_INTAKE = 0.2; 
const float WHEEL_DIAMETER = 4.125; // inches
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.1416;
const int TICKS_PER_REVOLUTION_18 = 900; // number of ticks per revolution for 18:1 gear ratio
const int TICKS_PER_LOOP = 450; // number of ticks to rotate the base motors in each loop
const int MAX_BASE_RETRIES_BEFORE_BREAKING_FROM_LOOP = 4; // number of retries before moveRobot breaks from while loop
const int MAX_SIDEDRIVE_RETRIES_BEFORE_BREAKING_FROM_LOOP = 2; // number of retries before moveSideDrive breaks from while loop
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
/*
    // 2a. Get the current motor encoder counter value
    double currentTicks = StackerMotor.rotation(vex::rotationUnits::raw)*direction;

    // 2b. Adjust the ticks based on the global encoder counter
    ticks = ticks - currentTicks;
*/

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
        } else if (remainingTicks < TICKS_BUFFER) {
          break;
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

void stopSpinningClaws()
{
  // Stop spinning the claws
    LeftClawMotor.stop();
    RightClawMotor.stop();

  // We are done stopping to spin the claw motors
}

void startMovingRobot(int speed, int direction) {
  // Set the velocity of the motors to 'speed'
  LeftFrontMotor.setVelocity(speed, velocityUnits::pct);
  RightFrontMotor.setVelocity(speed, velocityUnits::pct);
  LeftBackMotor.setVelocity(speed, velocityUnits::pct);
  RightBackMotor.setVelocity(speed, velocityUnits::pct);

  // Spin forward
  if (direction > 0) {
    LeftFrontMotor.spin(forward);
    RightFrontMotor.spin(forward);
    LeftBackMotor.spin(forward);
    RightBackMotor.spin(forward);
  }
  else if (direction < 0) {
    LeftFrontMotor.spin(reverse);
    RightFrontMotor.spin(reverse);
    LeftBackMotor.spin(reverse);
    RightBackMotor.spin(reverse);
  }
}

void stopMovingRobot() 
{
    LeftFrontMotor.stop();
    RightFrontMotor.stop();
    LeftBackMotor.stop();
    RightBackMotor.stop();
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

 void turnRobotHeading(double deg, int speed)
{
   // 0 = don't move, 1 = move clockwise, -1 = move counter-clockwise
    int moveClockwise = 0; 

    if (deg < 0) {
      deg = 360 + deg;
    }

    // Initializing Robot Configuration. DO NOT REMOVE!
    LeftBackMotor.setVelocity(speed,percent);
    LeftFrontMotor.setVelocity(speed,percent);
    RightFrontMotor.setVelocity(speed,percent);
    RightBackMotor.setVelocity(speed,percent);
  
    double initialGyroHeading = GyroSensor.heading(degrees);

    double degToMove = deg - initialGyroHeading;
    if (degToMove < -180) {
      degToMove = 360 + degToMove;
    }
    else if (degToMove > 180) {
      degToMove = degToMove - 360;
    }
  
  if (degToMove > 0) {
    // turn clockwise (right)
    moveClockwise = 1;
    LeftBackMotor.spin(forward);
    LeftFrontMotor.spin(forward);
    RightFrontMotor.spin(reverse);
    RightBackMotor.spin(reverse);
  }
  else if (degToMove < 0) {
    // turn counter-clockwise (left)
    moveClockwise = -1;
    LeftBackMotor.spin(reverse);
    LeftFrontMotor.spin(reverse);
    RightFrontMotor.spin(forward); 
    RightBackMotor.spin(forward);
  }
  else {
    // the robot is already at the right angle. No need to turn.
    moveClockwise = 0;
  }

  // Wait until the motors turn to get the robot at an angle of 'deg'
  if (moveClockwise == 1) {
    bool keepMoving = true;
    while (keepMoving) {
      double gyroHeading = GyroSensor.heading(degrees);
      double remainingDegrees = deg - gyroHeading;
      if (remainingDegrees < -180) {
        remainingDegrees = 360 + remainingDegrees;
      }
      else if (remainingDegrees > 180) {
        remainingDegrees = remainingDegrees - 360;
      }
      if (remainingDegrees <= 0) {
        keepMoving = false;
      }
    }
  } else if (moveClockwise == -1) {
    bool keepMoving = true;
    while (keepMoving) {
      double gyroHeading = GyroSensor.heading(degrees);
      double remainingDegrees = deg - gyroHeading;
      if (remainingDegrees < -180) {
        remainingDegrees = 360 + remainingDegrees;
      }
      else if (remainingDegrees > 180) {
        remainingDegrees = remainingDegrees - 360;
      }
      if (remainingDegrees >= 0) {
        keepMoving = false;
      }
    }

  }

  if ((moveClockwise == 1) || (moveClockwise == -1)) {
    LeftFrontMotor.stop();
    RightFrontMotor.stop();
    LeftBackMotor.stop();
    RightBackMotor.stop();
 }
}


void turnRobot(double deg, int speed, int direction)
 {
  // Initializing Robot Configuration. DO NOT REMOVE!
  LeftBackMotor.setVelocity(speed,percent);
  LeftFrontMotor.setVelocity(speed,percent);
  RightFrontMotor.setVelocity(speed,percent);
  RightBackMotor.setVelocity(speed,percent);

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

/*
    // 2a. Get the motor encoder counter
    double currentTicks = ArmMotor.rotation(vex::rotationUnits::raw)*direction;

    // 2b. Calculate the ticks to move
    ticks = ticks = currentTicks;
*/
  
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

void startSideDrive(int direction, int speed)
{
      LeftFrontMotor.setVelocity(speed,percent);
      LeftBackMotor.setVelocity(speed, percent);
      RightFrontMotor.setVelocity(speed, percent);
      RightBackMotor.setVelocity(speed, percent);

//right
    if(direction==1)
    {
      LeftFrontMotor.spin(forward);
      RightBackMotor.spin(forward);
      LeftBackMotor.spin(reverse);
      RightFrontMotor.spin(reverse);   
    }
    else{
//left
      LeftFrontMotor.spin(reverse);
      RightBackMotor.spin(reverse);
      LeftBackMotor.spin(forward);
      RightFrontMotor.spin(forward);
    }
}

void stopSideDrive()
{
  LeftFrontMotor.stop();
  RightBackMotor.stop();
  LeftBackMotor.stop();
  RightFrontMotor.stop();
}

void setStartingPosition() 
{
  // Raise the arms
  ArmMotor.setVelocity(100, velocityUnits::pct); 
  ArmMotor.rotateFor(500, vex::rotationUnits::deg, false);

  // Begin the outtake claws
  startSpinningClaws(1, 100);
  wait(1.75, sec);

  // Stop spinning claws
  stopSpinningClaws();

  // Bring the arms back to the starting position
  ArmMotor.rotateFor(-500, vex::rotationUnits::deg);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  GyroSensor.calibrate();
  while (GyroSensor.isCalibrating()) {
    wait(100, msec);
  }
  GyroSensor.setHeading(0, degrees);

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
  /*
  turnRobotHeading(90, 25);
  wait(3, sec);
  turnRobotHeading(0, 25);
  wait(3, sec);
*/
  // ..........................................................................
  //setStartingPosition();
  //startSpinningClaws(-1, 100);

  // Pick up the pre-load
  //wait(WAIT_FOR_CUBE_INTAKE, sec);

  // waits for the Inertial Sensor to calibrate
  /*while (GyroSensor.isCalibrating()) {
    wait(100, msec);
  }*/
  
  //moveRobot(22,50,1);
  //stopSpinningClaws();

  // first move right for 10 inches at 50% speed
 // moveSideDrive(24,25,1);

 
  setStartingPosition();

  startSpinningClaws(-1, 100);

  // Pick up the pre-load
  // wait(WAIT_FOR_CUBE_INTAKE, sec);
  
  moveRobot(12,50,1);
  moveRobot(6,25,1); 
  
  // Pick up cube #1
  wait(WAIT_FOR_CUBE_INTAKE, sec);
  for (int i = 0; i < 2; i++){
    moveRobot(6,25,1);
    // Pick up cube #2 & #3
    wait(WAIT_FOR_CUBE_INTAKE,sec);
  }
  wait(600, msec);
  moveRobot(9,25,1);

  // Pick up cube #4
  wait(300, msec);


  moveRobot(27,50,-1);

  moveSideDrive(28,25,-1);
  turnRobotHeading(0,50);
  wait(0.5, sec);
  moveRobot(11,25,1); 
  //Picking up second stack
  // Pick up cube #5
  wait(WAIT_FOR_CUBE_INTAKE, sec);
  for (int i = 0; i < 1; i++){
    moveRobot(6,25,1);
    // Pick up cube #6 & #7
    wait(WAIT_FOR_CUBE_INTAKE,sec);
  }
  wait(600, msec);
  moveRobot(10,25,1);
  stopSpinningClaws();

  moveRobot(35,50,-1);
  turnRobotHeading(90,50);
  moveRobot(23,50,1);
  startSideDrive(1,100);
  wait(1, sec);
  stopSideDrive();
  //moveSideDrive(26, 25, 1);
  //turnRobot(75,50,1);
  //turnRobotHeading(90,25,1);

  //moveRobot(10,50,1);
  //turnRobot(1,25,-1);
  //turnRobotHeading(70,25,-1);

  LeftFrontMotor.setVelocity(25, percent);
  RightFrontMotor.setVelocity(25, percent);
  LeftBackMotor.setVelocity(25, percent);
  RightBackMotor.setVelocity(25, percent);  

  LeftFrontMotor.spin(forward);
  RightFrontMotor.spin(forward);
  LeftBackMotor.spin(forward);
  RightBackMotor.spin(forward);   

  waitUntil(BumperA.pressing());
  
  LeftFrontMotor.stop();
  RightFrontMotor.stop();
  LeftBackMotor.stop();
  RightBackMotor.stop(); 

  startSpinningClaws(1,5);

  moveStacker(750, 25, 1);

  stopSpinningClaws();

  wait(1, sec);

  moveStacker(750, 25, -1);
  startSpinningClaws(1,25);
  wait(0.8, sec);
  stopSpinningClaws();

  moveRobot(12,25,-1);
// done stacking 7 stack

  turnRobotHeading(0,50);
  moveRobot(40,50,1);




  startSpinningClaws(-1, 50);
  wait(1.5, sec);
  stopSpinningClaws();
  wait(3, sec);
  //moveRobot(2,50,-1);
  wait(3, sec);
  moveArms(450,50,1);
  moveRobot(7,50,1);
  startSpinningClaws(1,50);
  wait(1, sec);
  stopSpinningClaws();
  // next move left for 10 inches at 25% speed
  

  // ..........................................................................
/*
  // An instance of brain used for printing to the V5 Brain screen
  brain  Brain;

  // VEXcode device constructors
  motor RightFrontBaseMotor = motor(PORT9, ratio18_1, false);
  motor LeftFrontBaseMotor = motor(PORT10, ratio18_1, false);
  motor LeftDriveSmart = motor(PORT8, ratio18_1, false);
  motor RightDriveSmart = motor(PORT1, ratio18_1, true);
 
  //drivetrain Drivetrain = drivetrain(LeftFrontBaseMotor, RightFrontBaseMotor, 319.19, 295, 130, mm, 1);

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
  */
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
//button X is now forward button
//Button B is now stacker automatic
//Button Y is stacker forward manual 
//Button A is stacker backward manual
void usercontrol(void) {
  //R2 & L2 are for sideDrive
  //R1 & L1 are for claws
  // An instance of brain used for printing to the V5 Brain screen
  brain  Brain;

  // VEXcode device constructors
  motor LeftBackDriveSmart = motor(PORT8, ratio18_1, false);
  motor LeftFrontDriveSmart = motor(PORT10, ratio18_1, false);
  drivetrain LeftDrivetrain = drivetrain(LeftBackDriveSmart, LeftFrontDriveSmart, 319.19, 295, 130, mm, 1);

  motor RightBackDriveSmart = motor(PORT1, ratio18_1, true);
  motor RightFrontDriveSmart = motor(PORT9, ratio18_1, true);
  drivetrain RightDrivetrain = drivetrain(RightBackDriveSmart, RightFrontDriveSmart, 319.19, 295, 130, mm, 1);

  // VEXcode generated functions
  // define variables used for controlling motors based on controller inputs
  //bool Controller1LeftShoulderControlMotorsStopped = true;
  //bool Controller1RightShoulderControlMotorsStopped = true;
  bool Controller1R2ButtonMotorsStopped = true;
  bool Controller1L1ButtonMotorsStopped = true;
  bool Controller1R1ButtonMotorsStopped = true;
  bool Controller1L2ButtonMotorsStopped = true;
  bool Controller1UpDownButtonsControlMotorsStopped = true;
  bool Controller1XBButtonsControlMotorsStopped = true;
  bool ControllerLeftButton = true;
  bool ControllerRightButton = false;
  bool ControllerXButton = true;
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

    if (Controller1.ButtonR2.pressing()) {    
      LeftFrontMotor.setVelocity(100,percent);
      LeftBackMotor.setVelocity(100, percent);
      RightFrontMotor.setVelocity(100, percent);
      RightBackMotor.setVelocity(100, percent);

      LeftFrontMotor.spin(forward);
      RightBackMotor.spin(forward);
      LeftBackMotor.spin(reverse);
      RightFrontMotor.spin(reverse);
      
      Controller1R2ButtonMotorsStopped = false;
    } else if (!Controller1R2ButtonMotorsStopped) {
       LeftFrontMotor.stop();
       RightBackMotor.stop();
       LeftBackMotor.stop();
       RightFrontMotor.stop();

       // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
       Controller1R2ButtonMotorsStopped = true;
    }
    
   
    if (Controller1.ButtonL2.pressing()) {         
      LeftFrontMotor.setVelocity(100,percent);
      LeftBackMotor.setVelocity(100, percent);
      RightFrontMotor.setVelocity(100, percent);
      RightBackMotor.setVelocity(100, percent);

      LeftFrontMotor.spin(reverse);
      RightBackMotor.spin(reverse);
      LeftBackMotor.spin(forward);
      RightFrontMotor.spin(forward);

      Controller1L2ButtonMotorsStopped = false;
    } else if (!Controller1L2ButtonMotorsStopped) {
      LeftFrontMotor.stop();
      RightBackMotor.stop();
      LeftBackMotor.stop();
      RightFrontMotor.stop();

      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1L2ButtonMotorsStopped = true;
    }
    

    if (Controller1.ButtonL1.pressing()) {
      RightClawMotor.setVelocity(100, pct);
      LeftClawMotor.setVelocity(100, pct);
      
      RightClawMotor.spin(forward);
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
      LeftClawMotor.setVelocity(100, pct);
      
      RightClawMotor.spin(reverse);
      LeftClawMotor.spin(reverse);
          
      Controller1R1ButtonMotorsStopped = false;
    } else if (!Controller1R1ButtonMotorsStopped) {
      RightClawMotor.stop();
      LeftClawMotor.stop();
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1R1ButtonMotorsStopped = true;
    }
//XX
    if (Controller1.ButtonB.pressing()) {
      int countDegrees = 0;
      //double currentTicks = StackerMotor.rotation(vex::rotationUnits::raw);
      //logText("curr: ", currentTicks);
      moveStacker(500, 50, 1);

      while (countDegrees < 500) {
        moveStacker(100, 50, 1);
        moveClaws(50,50,-1);
        countDegrees += 100;
      }
      moveStacker(125, 20, 1);
      ControllerYButton = false;
    } else if (!ControllerYButton){
      StackerMotor.stop();
      StackerMotor.setBrake(hold);
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      ControllerYButton = true;
    }

    if (Controller1.ButtonX.pressing()) {
        LeftFrontDriveSmart.setVelocity(50, pct);
        RightFrontDriveSmart.setVelocity(50, pct);
        RightBackDriveSmart.setVelocity(50, pct);
        LeftBackDriveSmart.setVelocity(50, pct);
        LeftFrontDriveSmart.spin(forward);
        RightFrontDriveSmart.spin(forward);
        RightBackDriveSmart.spin(forward);
        LeftBackDriveSmart.spin(forward);
    
        ControllerXButton = false;

      } else if (!ControllerXButton) {
        LeftFrontDriveSmart.stop();
        RightFrontDriveSmart.stop();
        RightBackDriveSmart.stop();
        LeftBackDriveSmart.stop();
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      ControllerXButton = true;
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
        if (ControllerRightButton == false){
          setStartingPosition();
          ControllerRightButton = true;
        }
      }

    // check the Up/Down Buttons status to control ArmMotor
    if (Controller1.ButtonUp.pressing()) {
      ArmMotor.setVelocity(100,percent);
      ArmMotor.spin(forward);
      Controller1UpDownButtonsControlMotorsStopped = false;
    } else if (Controller1.ButtonDown.pressing()) {
      ArmMotor.setVelocity(100,percent);
      ArmMotor.spin(reverse);
      Controller1UpDownButtonsControlMotorsStopped = false;
    } else if (!Controller1UpDownButtonsControlMotorsStopped){
      ArmMotor.stop();
      ArmMotor.setBrake(hold);
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1UpDownButtonsControlMotorsStopped = true;
    }

    // check the X/B buttons status to control StackerMotor
    if (Controller1.ButtonY.pressing()) {
      StackerMotor.setVelocity(100, percentUnits::pct);
      StackerMotor.spin(forward);
      Controller1XBButtonsControlMotorsStopped = false;
    } else if (Controller1.ButtonA.pressing()) {
      StackerMotor.setVelocity(100, percentUnits::pct);
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
