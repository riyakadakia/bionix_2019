
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
  
  // Lift the arm to unsnap the stacker
  ArmMotor.spinFor(1, rotationUnits::rev, 100, velocityUnits::pct);
  ArmMotor.spinFor(-1, rotationUnits::rev, 100, velocityUnits::pct);

  // Spin the claw motors to unsnap the arms
  LeftClawMotor.spinFor(1, rotationUnits::rev, 100, velocityUnits::pct);
  RightClawMotor.spinFor(1, rotationUnits::rev, 100, velocityUnits::pct);

  // Start spinning the claws, get ready to intake cubes
  LeftClawMotor.spin(directionType::rev, 100, percentUnits::pct);
  RightClawMotor.spin(directionType::rev, 100, percentUnits::pct);

  // Pick up the pre-load
  wait(0.25, timeUnits::sec);

  // Move forward to pick up the other 4 cubes
  Drivetrain.driveFor(directionType::fwd, 36, distanceUnits::in, 25, velocityUnits::pct);
  // wait(0.25, timeUnits::sec);

  // Turn around to move back to place the cubes in the goal
  Drivetrain.turnFor(160, rotationUnits::deg, 25, velocityUnits::pct);
  Drivetrain.driveFor(directionType::fwd, 28, distanceUnits::in, 50, velocityUnits::pct);

  // Wait to pick up the last cube
  wait(0.5, timeUnits::sec);

  // Stop the claws from spinning
  LeftClawMotor.stop();
  RightClawMotor.stop();

  // Spin the claws out to move the bottom-most cube out to place in the goal
 // LeftClawMotor.spinFor(0.25, rotationUnits::rev, 25, velocityUnits::pct);
 // RightClawMotor.spinFor(0.25, rotationUnits::rev, 25, velocityUnits::pct);

  // Angle the stacker to place the cubes in the goal
  Stacker.spinFor(0.5, rotationUnits::rev, 25, velocityUnits::pct);

  LeftClawMotor.spin(directionType::fwd, 25, percentUnits::pct);
  RightClawMotor.spin(directionType::fwd, 25, percentUnits::pct);
  wait(500, timeUnits::msec);
  LeftClawMotor.stop();
  RightClawMotor.stop();

  // Angle the stacker to place the cubes in the goal
  Stacker.spinFor(0.8, rotationUnits::rev, 25, velocityUnits::pct);

  // Move the robot forward to the goal
  Drivetrain.driveFor(directionType::fwd, 1, distanceUnits::in, 5, velocityUnits::pct);

  // Wait to place the cubes
  // wait(0.25, timeUnits::sec);

  // Move the robot back from the goal
  Drivetrain.driveFor(directionType::rev, 12, distanceUnits::in, 15, velocityUnits::pct);

  //Stacker.spinFor(-1.1, rotationUnits::rev, 25, velocityUnits::pct);
}
