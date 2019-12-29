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

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // setup controller lcd
 /* Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("HELLO!");
  Controller1.Screen.newLine();
  Controller1.Screen.print("WORLD!");
  wait(1,seconds);*/
    
  // set the initial rotation value for all motor encoders to 0
  LeftFrontMotor.setRotation(0, vex::rotationUnits::raw);
  LeftBackMotor.setRotation(0, vex::rotationUnits::raw);
  RightFrontMotor.setRotation(0, vex::rotationUnits::raw);
  RightBackMotor.setRotation(0, vex::rotationUnits::raw);
  ArmMotor.setRotation(0, vex::rotationUnits::raw);

  /*Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Cleared the encoder rotation value of all motors");
  wait(1,seconds);*/

// setting the velocity/ speed for all the motors
  LeftFrontMotor.setVelocity(75, velocityUnits::pct);
  RightFrontMotor.setVelocity(75, velocityUnits::pct);
  LeftBackMotor.setVelocity(75, velocityUnits::pct);
  RightBackMotor.setVelocity(75, velocityUnits::pct);
  ArmMotor.setVelocity(75, velocityUnits::pct);
  LeftClawMotor.setVelocity(100, velocityUnits::pct);
  RightClawMotor.setVelocity(100, velocityUnits::pct);

  //Controller1.Screen.clearScreen();
  //Controller1.Screen.setCursor(1,1);
  //Controller1.Screen.print("Set the velocity of all motors to 75%");
  //wait(1,seconds);

  double ticksLFM = 0;
  double ticksRFM = 0;
  double ticksLBM = 0;
  double ticksRBM = 0;
  double ticksAM = 0;


moveRobot(15, 100);

void moveRobot(float inches, int speed) 
{
    // 1. Convert inches into ticks (based on gear ratio)
    //    - 900 ticks for 18:1 gear ratio
    //    - 1,800 ticks for 36:1 gear ratio
    var diameter_of_wheel = 4;
    double ticks = 900*inches/(diameter_of_wheel*3.14);

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

            // calculate remaining_ticks to move
            double remaining_ticks = ticks - ticksLFM;

            // if remaining_ticks > 50, set moveTicks (variable) = 50
            // else set moveTicks = remaining_ticks
            if (remaining_ticks > 50) {
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

            // calculate remaining_ticks to move
            double remaining_ticks = ticks - ticksLBM;

            // if remaining_ticks > 50, set moveTicks (variable) = 50
            // else set moveTicks = remaining_ticks
            if (remaining_ticks > 50) {
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

            // calculate remaining_ticks to move
            double remaining_ticks = ticks - ticksRBM;

            // if remaining_ticks > 50, set moveTicks (variable) = 50
            // else set moveTicks = remaining_ticks
            if (remaining_ticks > 50) {
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

            // calculate remaining_ticks to move
            double remaining_ticks = ticks - ticksRFM;

            // if remaining_ticks > 50, set moveTicks (variable) = 50
            // else set moveTicks = remaining_ticks
            if (remaining_ticks > 50) {
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
  
  /*while (ticksLFM < 20000) {
    LeftFrontMotor.rotateFor(450, vex::rotationUnits::raw, false);
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    ticksLFM = LeftFrontMotor.rotation(vex::rotationUnits::raw);
    Controller1.Screen.print("LFM: %lf", ticksLFM);   
    //wait(5,seconds);

    RightFrontMotor.rotateFor(450, vex::rotationUnits::raw, false);
    Controller1.Screen.setCursor(2,1);
    ticksRFM = RightFrontMotor.rotation(vex::rotationUnits::raw);
    Controller1.Screen.print("RFM: %lf", ticksRFM);
    //wait(5,seconds);

    LeftBackMotor.rotateFor(450, vex::rotationUnits::raw, false);
    Controller1.Screen.setCursor(3,1);
    ticksLBM = LeftBackMotor.rotation(vex::rotationUnits::raw);
    Controller1.Screen.print("LBM: %lf", ticksLBM);
    //wait(5,seconds);

    RightBackMotor.rotateFor(450, vex::rotationUnits::raw, false);
    Controller1.Screen.setCursor(4,1);
    ticksRBM = RightBackMotor.rotation(vex::rotationUnits::raw);
    Controller1.Screen.print("RBM: %lf", ticksRBM);
    //wait(15,seconds);
  }

  wait(1, timeUnits::sec);
  */

    LeftFrontMotor.rotateFor(450, vex::rotationUnits::raw, false);
    RightFrontMotor.rotateFor(450, vex::rotationUnits::raw, false);
    LeftBackMotor.rotateFor(450, vex::rotationUnits::raw, false);
    RightBackMotor.rotateFor(450, vex::rotationUnits::raw, true);
    
    ArmMotor.rotateFor(2700, vex::rotationUnits::raw, true);//2700
    LeftClawMotor.rotateFor(-1200, vex::rotationUnits::raw, false);//1200
    RightClawMotor.rotateFor(1200, vex::rotationUnits::raw, true);
    ArmMotor.rotateFor(-2700, vex::rotationUnits::raw, true);//2700
 
    time(&done);
    time_t it_took = done - now;
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Time: %ld", it_took);
    wait(10,seconds);
}
