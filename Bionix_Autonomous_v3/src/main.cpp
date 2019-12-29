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
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("HELLO!");
  Controller1.Screen.newLine();
  Controller1.Screen.print("WORLD!");
  wait(1,seconds);
    
  // set the initial rotation value for all motor encoders to 0
  LeftFrontMotor.setRotation(0, vex::rotationUnits::raw);
  LeftBackMotor.setRotation(0, vex::rotationUnits::raw);
  RightFrontMotor.setRotation(0, vex::rotationUnits::raw);
  RightBackMotor.setRotation(0, vex::rotationUnits::raw);
  
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Cleared the encoder rotation value of all motors");
  wait(1,seconds);

  LeftFrontMotor.setVelocity(25, velocityUnits::pct);
  RightFrontMotor.setVelocity(25, velocityUnits::pct);
  LeftBackMotor.setVelocity(25, velocityUnits::pct);
  RightBackMotor.setVelocity(25, velocityUnits::pct);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Set the velocity of all motors to 50%");
  wait(1,seconds);

  double ticksLFM = 0;
  double ticksRFM = 0;
  double ticksLBM = 0;
  double ticksRBM = 0;
  
  while (ticksLFM < 20000) {
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
  

}
