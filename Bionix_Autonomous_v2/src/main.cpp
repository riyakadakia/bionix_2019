
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
// Controller1          controller                    
// RightClawMotor       motor         3               
// Drivetrain           drivetrain    2, 1            
// ArmMotor             motor         6               
// StackerMotor         motor         7               
// LeftClawMotor        motor         8               
// RightFrontBaseMotor  motor         9               
// LeftFrontBaseMotor   motor         10              
#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    2, 1            
// RightClawMotor       motor         3               
// ArmMotor             motor         6               
// Stacker              motor         7               
// LeftClawMotor        motor         8               
// RightFrontBaseMotor  motor         9               
// LeftFrontBaseMotor   motor         10              
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  //Drivetrain.driveFor(reverse, 32, distanceUnits::in);
  //wait(1, timeUnits::sec);
  
  ArmMotor.spinFor(1, rotationUnits::rev, 100, velocityUnits::pct);
  ArmMotor.spinFor(-1, rotationUnits::rev, 100, velocityUnits::pct);
  LeftClawMotor.spinFor(1, rotationUnits::rev, 100, velocityUnits::pct);
  RightClawMotor.spinFor(1, rotationUnits::rev, 100, velocityUnits::pct);

  LeftClawMotor.spin(directionType::rev, 100, percentUnits::pct);
  RightClawMotor.spin(directionType::rev, 100, percentUnits::pct);
  wait(0.5, timeUnits::sec);
  Drivetrain.driveFor(directionType::fwd, 36, distanceUnits::in, 25, velocityUnits::pct);

}
