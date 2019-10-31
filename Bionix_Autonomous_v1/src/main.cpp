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
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Drivetrain.driveFor(reverse, 15, distanceUnits::in);
  wait(1, timeUnits::sec);
  Drivetrain.driveFor(forward, 15, distanceUnits::in);
  wait(1, timeUnits::sec);

}
