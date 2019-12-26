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

const float WHEEL_DIAMETER = 4.125; // inches
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.1416;
const int TICKS_PER_REVOLUTION_36 = 1800; // number of ticks per revolution for 36:1 gear ratio
const int TICKS_PER_REVOLUTION_18 = 900; // number of ticks per revolution for 18:1 gear ratio
const float TURNING_DIAMETER = 17.5; // turning diameter or diagonal distance between the wheels
const int TICKS_PER_LOOP = 450; // number of ticks to rotate the base motors in each loop
const int TICKS_PER_LOOP_STACKER = 50; // number of ticks to rotate the stacker motor in each loop
const int TICKS_PER_LOOP_ARM = 50; //number of ticks to rotate the arm motor in each loop

/**
  convertInchesIntoTicks_18 converts 'inches' into the appropriate number of motor ticks
  depending on the ticks for motor with gear ratio of 18:1 and wheel of 4.125" diameter
**/
double convertInchesIntoTicks_18(float inches) {
    // Convert inches into ticks (based on gear ratio)
    // 900 ticks per revolution for 18:1 gear ratio
    double ticks = TICKS_PER_REVOLUTION_18*inches/WHEEL_CIRCUMFERENCE;
    return ticks;
}

/**
  convertInchesIntoTicks_36 converts 'inches' into the appropriate number of motor ticks
  depending on the ticks for motor with gear ratio of 36:1 and wheel of 4.125" diameter
**/
double convertInchesIntoTicks_36(float inches) {
    // Convert inches into ticks (based on gear ratio)
    // 1,800 ticks per revolution for 18:1 gear ratio
    double ticks = TICKS_PER_REVOLUTION_36*inches/WHEEL_CIRCUMFERENCE;
    return ticks;
}

/**
  convertDegreesIntoTicks_18 converts 'degrees' into the appropriate number of motor ticks
  depending on the ticks for motor with gear ratio of 18:1 and wheel of 4.125" diameter
**/
int convertDegreesIntoTicks_18(float degrees) {
    // Convert degrees into ticks (based on gear ratio)
    // 900 ticks per revolution for 18:1 gear ratio
    double ticks = TICKS_PER_REVOLUTION_18*degrees/360;
    return ticks;  
}

/**
  convertDegreesIntoTicks_36 converts 'degrees' into the appropriate number of motor ticks
  depending on the ticks for motor with gear ratio of 36:1 and wheel of 4.125" diameter
**/
int convertDegreesIntoTicks_36(float degrees) {
    // Convert degrees into ticks (based on gear ratio)
    // 1,800 ticks per revolution for 36:1 gear ratio
    double ticks = TICKS_PER_REVOLUTION_36*degrees/360;
    return ticks;  
}

/**
**/
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
   moveArms moves the arms up and down by 'degrees' and 
   at a certain 'speed' pct. Use positive number in degrees
   to move up and negative number to move down. ArmMotor uses
   a cartrigde with 36:1 gear ratio.
**/
void moveArms(float degrees, int speed)
{
    // 1. Convert degrees into ticks
    double ticks = convertDegreesIntoTicks_36(degrees);

    // 2. Set the initial motor encoder counter to 0
    ArmMotor.setRotation(0, vex::rotationUnits::raw);
  
    // 3. Set the velocity of the motor to 'speed'
    ArmMotor.setVelocity(speed, velocityUnits::pct);

    // 4. Create counter variable and set it to 0
    double ticksAM = 0;

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

/** 
   moveStacker moves the stacker up and down by 'degrees' and 
   at a certain 'speed' pct. Use positive number in degrees
   to move up and negative number to move down. StackerMotor uses
   a cartrigde with 36:1 gear ratio.
**/
void moveStacker(float degrees, int speed)
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
    while (ticksSM < ticks && !StackerMotor.isSpinning()) {
      
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

/** 
   moveClaws spins both the claws in and out by 'degrees' and 
   at a certain 'speed' pct. Use positive number in degrees
   to move in and negative number to move out. Claw motors use
   a cartrigde with 18:1 gear ratio.
**/
void moveClaws(float degrees, int speed) 
{
    // 1. Convert inches into ticks
    double ticks = convertDegreesIntoTicks_18(degrees);

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

          if (ticksLCM < ticks && !LeftClawMotor.isSpinning()) {
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

          if (ticksRCM < ticks && !RightClawMotor.isSpinning()) {
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

/** 
   moveRobot moves all 4 base motors forward and backward by 'inches' 
   and at a certain 'speed' pct. Use positive number in inches to 
   move forward and negative number to move backward. Base motors use
   a cartrigde with 18:1 gear ratio.
**/
void moveRobot(float inches, int speed) 
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

    // 5. Loop - while (any counter variable < ticks)
    while (ticksLFM < ticks || ticksRFM < ticks || 
           ticksLBM < ticks || ticksRBM < ticks) {
      
          if (ticksLFM < ticks) {
            double moveLFMTicks = 0;

            // read the counter value
            ticksLFM = LeftFrontMotor.rotation(vex::rotationUnits::raw);

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
            LeftFrontMotor.rotateFor(moveLFMTicks, vex::rotationUnits::raw, false);
          }

          if (ticksLBM < ticks) {
            double moveLBMTicks = 0;

            // read the counter value
            ticksLBM = LeftBackMotor.rotation(vex::rotationUnits::raw);

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
            LeftBackMotor.rotateFor(moveLBMTicks, vex::rotationUnits::raw, false);
          }

          if (ticksRBM < ticks) {
            double moveRBMTicks = 0;

            // read the counter value
            ticksRBM = RightBackMotor.rotation(vex::rotationUnits::raw);

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
            RightBackMotor.rotateFor(moveRBMTicks, vex::rotationUnits::raw, false);
          }   

          if (ticksRFM < ticks) {
            double moveRFMTicks = 0;

            // read the counter value
            ticksRFM = RightFrontMotor.rotation(vex::rotationUnits::raw);

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
            RightFrontMotor.rotateFor(moveRFMTicks, vex::rotationUnits::raw, false);
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
          wait(TICKS_PER_LOOP*33.333/speed, msec);
    }
    // We are done moving the robot
}

/** 
   startSpinningClaws starts spinning the claw motors either for intake or
   to push the cubes out at a certain 'speed' pct. Set the 'direction' = 1 
   to move cubes in and set 'direction' = 2 to move the cubes out. If the
   claws are already spinning, it will do nothing
**/
void startSpinningClaws(int direction, int speed)
{
  // 1. Set the speed for both the claws
  if (!LeftClawMotor.isSpinning()) {
    LeftClawMotor.setVelocity(speed, vex::velocityUnits::pct);
  }
  if (!RightClawMotor.isSpinning()) {
    RightClawMotor.setVelocity(speed, vex::velocityUnits::pct);
  }

  // 2. Now begin spinning the claws
  if (!LeftClawMotor.isSpinning() && direction == 1) {
    LeftClawMotor.spin(forward);
  } else if (!LeftClawMotor.isSpinning() && direction == 2) {
    LeftClawMotor.spin(reverse);
  }

  if (!RightClawMotor.isSpinning() && direction == 1) {
    RightClawMotor.spin(reverse);
  } else if (!RightClawMotor.isSpinning() && direction == 2) {
    RightClawMotor.spin(forward);
  }

  // We are done starting to spin the claw motors at the 'speed'
}

/** 
   stopSpinningClaws stop spinning the claw motors. If the
   claws are not spinning, it will do nothing
**/
void stopSpinningClaws()
{
  // Stop spinning the claws
  if (LeftClawMotor.isSpinning()) {
    LeftClawMotor.stop();
  }

  if (RightClawMotor.isSpinning()) {
    RightClawMotor.stop();
  }

  // We are done stopping to spin the claw motors
}

/**
  turnRobot turns the robot by 'degrees'. It does so by turning its front two motors at a certain 'speed'. 
  In order to move left (negative degrees), it moves the left front wheel back by degrees/2 and right 
  front wheel forward by degrees/2. To move right (positive degrees), it moves the left front wheel forward 
  by degrees/2 and right front wheel back by degrees/2. Note: +90 degrees is a right turn
**/
void turnRobot(float degrees, int speed) 
{
    // A four-wheeled robot has a turning diameter approximated by the diagonal distance 
    // between the wheels. This distance can simply be measured on the robot as the distance 
    // between the upper left and lower right wheels at the points where they contact the
    // ground  
    float turningRatio = TURNING_DIAMETER / WHEEL_DIAMETER;
    float wheelDegrees = turningRatio * degrees; 

    // Divide by two because each wheel provides half the rotation
    LeftFrontMotor.startRotateFor(wheelDegrees * 1/2, vex::rotationUnits::deg, speed, vex::velocityUnits::pct);
    RightFrontMotor.rotateFor(wheelDegrees * 1/2, vex::rotationUnits::deg, speed, vex::velocityUnits::pct);
}

/**
  setStartingPosition prepares the robot to move into the starting 
  position after the timer is started
**/ 
void setStartingPosition() 
{
  // 1. Push the stacker out
  StackerMotor.rotateFor(200, vex::rotationUnits::deg, false);

  // 2. Even while the stacker is moving out, raise the arms
  ArmMotor.setVelocity(100, velocityUnits::pct); 
  ArmMotor.rotateFor(400, vex::rotationUnits::deg);

  // 3. Spin the claws outward to untangle the stacker if it gets stuck
  RightClawMotor.rotateFor(180, vex::rotationUnits::deg, false);
  LeftClawMotor.rotateFor(180, vex::rotationUnits::deg, true);

  // 4. Bring the arms back to the starting position
  ArmMotor.rotateFor(-400, vex::rotationUnits::deg);

  // 5. Bring the stacker back to the starting position
  StackerMotor.rotateFor(-200, vex::rotationUnits::deg, false);
}


int main() {
  vexcodeInit();
  
  // Set the starting position of the robot
  //setStartingPosition();

  // Move the robot 36" at 75% speed
  moveRobot(36, 20); 

  // Move the arm motor 30 degrees at 50% speed
  //moveArms(30, 50);

  // Move the claws 45 degrees at 80% speed
  //moveClaws(45, 80);
  
  // Move the stacker 30 degrees at 90% speed
  //moveStacker(30, 90);

  // Start spinning the claws for intake (1) at 80% speed
  //startSpinningClaws(1, 80);

  //wait(5, seconds);

  // Stop spinning the claws
  //stopSpinningClaws();

  // Turn the robot right by 45 degrees at 25% speed
  //turnRobot(45, 25);

}







