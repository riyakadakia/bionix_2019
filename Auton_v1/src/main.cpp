/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       riyakadakia                                               */
/*    Created:      Thu Oct 31 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightClawMotor       motor         3               
// ArmMotor             motor         6               
// StackerMotor         motor         7               
// LeftClawMotor        motor         8               
// RightFrontMotor      motor         9               
// LeftFrontMotor       motor         10              
// RightBackMotor       motor         1               
// LeftBackMotor        motor         2               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

// n = normal
//RightClawMotor 18:1 n 3
//ArmMotor 36:1 reverse 6
//StackerMotor 36:1 n 7
//LeftClawMotor 18:1 reverse 8
//RightFrontMotor 18:1 reverse 9
//LeftFrontMotor 18:1 n 10
//RightBackMotor 18:1 reverse 1
//LeftBackMotor 18:1 n 2

#include "vex.h"

using namespace vex;

double convertInchesIntoTicks(float inches) {
    // Convert inches into ticks (based on gear ratio)
    //    - 900 ticks for 18:1 gear ratio
    //    - 1,800 ticks for 36:1 gear ratio
    int diameter_of_wheel = 4;
    double ticks = 900*inches/(diameter_of_wheel*3.14);
    return ticks;
}

int convertDegreesIntoTicks(float degrees) {
    // Convert inches into ticks (based on gear ratio)
    //    - 900 ticks for 18:1 gear ratio
    //    - 1,800 ticks for 36:1 gear ratio
    double ticks = 900*degrees/360;
    return ticks;  
}

void moveArms(float degrees, int speed)
{
    // 1. Convert inches into ticks
    double ticks = convertDegreesIntoTicks(degrees);

    // 2. Set the initial motor encoder counters to 0
    ArmMotor.setRotation(0, vex::rotationUnits::raw);
  
    // 3. Set the velocity of the motor to 'speed'
    ArmMotor.setVelocity(speed, velocityUnits::pct);

    // 4. Create counter variable and set it to 0
    double ticksAM = 0;

    // 5. Loop - while (counter variable < ticks)
    while (ticksAM < ticks) {
      
          if (ticksAM < ticks) {
            double moveAMTicks = 0;

            // read the counter value
            ticksAM = ArmMotor.rotation(vex::rotationUnits::raw);

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksAM;

            // if remainingTicks > 50, set moveTicks (variable) = 50
            // else set moveTicks = remainingTicks
            if (remainingTicks > 50) {
              moveAMTicks = 50;
            } else {
              moveAMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            ArmMotor.rotateFor(moveAMTicks, vex::rotationUnits::raw, false);
          }
    }
    // We are done moving the arms  
}

void moveClaws(float degrees, int speed) 
{
    // 1. Convert inches into ticks
    double ticks = convertDegreesIntoTicks(degrees);

    // 2. Set the initial motor encoder counters to 0
    LeftClawMotor.setRotation(0, vex::rotationUnits::raw);
    RightClawMotor.setRotation(0, vex::rotationUnits::raw);
  
    // 3. Set the velocity of the motors to 'speed'
    LeftClawMotor.setVelocity(speed, velocityUnits::pct);
    RightClawMotor.setVelocity(speed, velocityUnits::pct);

    // 4. Create counter variables and set them to 0
    double ticksLCM = 0;
    double ticksRCM = 0;

    // 5. Loop - while (any counter variable < ticks)
    while (ticksLCM < ticks || ticksRCM < ticks) {   
          if (ticksLCM < ticks) {
            double moveLCMTicks = 0;

            // read the counter value
            ticksLCM = LeftClawMotor.rotation(vex::rotationUnits::raw);

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksLCM;

            // if remainingTicks > 50, set moveTicks (variable) = 50
            // else set moveTicks = remainingTicks
            if (remainingTicks > 50) {
              moveLCMTicks = 50;
            } else {
              moveLCMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            LeftClawMotor.rotateFor(moveLCMTicks, vex::rotationUnits::raw, false);
          }

          if (ticksRCM < ticks) {
            double moveRCMTicks = 0;

            // read the counter value
            ticksRCM = RightClawMotor.rotation(vex::rotationUnits::raw);

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksRCM;

            // if remainingTicks > 50, set moveTicks (variable) = 50
            // else set moveTicks = remainingTicks
            if (remainingTicks > 50) {
              moveRCMTicks = 50;
            } else {
              moveRCMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            RightClawMotor.rotateFor(moveRCMTicks, vex::rotationUnits::raw, false);
          }   
    }
    // We are done moving the claws
}

void moveRobot(float inches, int speed) 
{
    // 1. Convert inches into ticks
    double ticks = convertInchesIntoTicks(inches);

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

    // 5. Loop - while (any counter variable < ticks)
    while (ticksLFM < ticks || ticksRFM < ticks || 
           ticksLBM < ticks || ticksRBM < ticks) {
      
          if (ticksLFM < ticks) {
            double moveLFMTicks = 0;

            // read the counter value
            ticksLFM = LeftFrontMotor.rotation(vex::rotationUnits::raw);

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksLFM;

            // if remainingTicks > 50, set moveTicks (variable) = 50
            // else set moveTicks = remainingTicks
            if (remainingTicks > 50) {
              moveLFMTicks = 50;
            } else {
              moveLFMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            LeftFrontMotor.rotateFor(moveLFMTicks, vex::rotationUnits::raw, false);
          }

          if (ticksLBM < ticks) {
            double moveLBMTicks = 0;

            // read the counter value
            ticksLBM = LeftBackMotor.rotation(vex::rotationUnits::raw);

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksLBM;

            // if remainingTicks > 50, set moveTicks (variable) = 50
            // else set moveTicks = remainingTicks
            if (remainingTicks > 50) {
              moveLBMTicks = 50;
            } else {
              moveLBMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            LeftBackMotor.rotateFor(moveLBMTicks, vex::rotationUnits::raw, false);
          }

          if (ticksRBM < ticks) {
            double moveRBMTicks = 0;

            // read the counter value
            ticksRBM = RightBackMotor.rotation(vex::rotationUnits::raw);

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksRBM;

            // if remainingTicks > 50, set moveTicks (variable) = 50
            // else set moveTicks = remainingTicks
            if (remainingTicks > 50) {
              moveRBMTicks = 50;
            } else {
              moveRBMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            RightBackMotor.rotateFor(moveRBMTicks, vex::rotationUnits::raw, false);
          }   

          if (ticksRFM < ticks) {
            double moveRFMTicks = 0;

            // read the counter value
            ticksRFM = RightFrontMotor.rotation(vex::rotationUnits::raw);

            // calculate remainingTicks to move
            double remainingTicks = ticks - ticksRFM;

            // if remainingTicks > 50, set moveTicks (variable) = 50
            // else set moveTicks = remainingTicks
            if (remainingTicks > 50) {
              moveRFMTicks = 50;
            } else {
              moveRFMTicks = remainingTicks;
            }

            // rotateFor(moveTicks, raw, false)
            RightFrontMotor.rotateFor(moveRFMTicks, vex::rotationUnits::raw, false);
          }               
     
    }
    // We are done moving the robot
}

void setStartingPosition() {
  ArmMotor.setVelocity(100, velocityUnits::pct);  
  StackerMotor.rotateFor(200, vex::rotationUnits::deg, false);
  ArmMotor.rotateFor(400, vex::rotationUnits::deg);
  RightClawMotor.rotateFor(180, vex::rotationUnits::deg, false);
  LeftClawMotor.rotateFor(180, vex::rotationUnits::deg, true);
  ArmMotor.rotateFor(-400, vex::rotationUnits::deg);
  StackerMotor.rotateFor(-200, vex::rotationUnits::deg, false);
}

int main() {
  vexcodeInit();
  
  // Set the starting position of the robot
  setStartingPosition();

  // Move the robot 36" at 75% speed
  moveRobot(36, 75); 

  // Move the arm motor 180 degrees at 50% speed
  moveArms(180, 50);

  // Move the claws 45 degrees at 80% speed
  moveClaws(45, 80);
  
}







