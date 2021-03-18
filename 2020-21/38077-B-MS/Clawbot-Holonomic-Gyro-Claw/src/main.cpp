// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// back_right_motor     motor         12              
// back_left_motor      motor         11              
// front_right_motor    motor         2               
// front_left_motor     motor         1               
// Controller1          controller                    
// InertialSensor       inertial      16              
// LineTrackerA         line          A               
// LineTrackerB         line          B               
// LineTrackerC         line          C               
// MotorClaw            motor         20              
// MotorShoulder        motor         21              
// ---- END VEXCODE CONFIGURED DEVICES ----

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\msnyd                                            */
/*    Created:      Tue Oct 06 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <algorithm>
#include <cmath>
// #include <robot-config.h> ... should be included by vex.h
#include "screen-buttons.h"
#include "autonomous.h"
#include "motors.h"

using namespace vex;

// A global instance of competition
competition Competition;

// ******************************************************* Pre-auto ****************************************************

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // find zero position for claw by checking current draw
  // note: this motion is not allowed with field control
  MotorClaw.setMaxTorque(80, vex::percentUnits::pct);
  if (!Competition.isCompetitionSwitch() && !Competition.isFieldControl()) {
    Brain.Screen.clearScreen();
    //Brain.Screen.print("Zeroing claw!");
    //MotorClaw.setVelocity(75,vex::velocityUnits::pct);
    //MotorClaw.spin(vex::directionType::rev);
    //while((MotorClaw.current(vex::amp)<0.6)){
    //  vex::task::sleep(100);
    //  //Brain.Screen.print(MotorClaw.power());
    //}
    //MotorClaw.stop();
  }
  MotorClaw.setRotation(0,vex::deg);
  MotorClaw.setStopping(vex::brakeType::hold);

  // // find zero position for shoulder by hitting bumper
  // while(! BumperA.pressing()){
  //   MotorShoulder.spin(vex::directionType::rev); 
  // }
  // MotorShoulder.stop();
  MotorShoulder.setRotation(0,vex::rotationUnits::deg);
  MotorShoulder.setStopping(vex::brakeType::hold);
  MotorShoulder.setVelocity(75,vex::velocityUnits::rpm);

  // calibrate gyro
  Brain.Screen.clearScreen();
  Brain.Screen.print("Calibrating!");
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(50, 50, 400, 100);

  InertialSensor.startCalibration();
  while (InertialSensor.isCalibrating()) { task::sleep(50); }
  wait(1.0, seconds);

  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawCircle(250, 125, 75);
  Brain.Screen.setFillColor(transparent);

  if (Competition.isCompetitionSwitch() || Competition.isFieldControl()) {
    // on competition switch, run auton selection process
    DrawButtons();
    while (!Brain.Screen.pressing() && !Competition.isEnabled()) { task::sleep(50); } // exit loop when press or auto is enabled
    if (Brain.Screen.pressing()) { AutonChoice = AutonLocation(); }
    else AutonChoice = AUTON_DO_NOTHING;
    // while (Brain.Screen.pressing()) task::sleep(50);
    HiliteTouch();
  }

  pre_auton_done = true;
}

// *********************************************** User Control **************************************************

void usercontrol() {

  while (!pre_auton_done) {
    vex::task::sleep(100);
  }

  Brain.Screen.clearScreen();
  Brain.Screen.print("Driver Control");

  while(true) {

    if (Controller1.ButtonB.pressing()) {  // reset 0 heading to current heading
      InertialSensor.setHeading(0, degrees);
      InertialSensor.setRotation(0, degrees);
    }

    // Arm Control
    if((Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing())
    || (Controller2.ButtonR1.pressing() && Controller2.ButtonR2.pressing())) { // middle position
          MotorShoulder.setVelocity(SHOULDER_SPEED_UP, vex::velocityUnits::pct);
          MotorShoulder.startRotateTo(SHOULDER_POS_MID,vex::rotationUnits::deg);
    }
    else if(Controller1.ButtonR1.pressing()
         || Controller2.ButtonR1.pressing()) { // highest position
          MotorShoulder.setVelocity(SHOULDER_SPEED_UP, vex::velocityUnits::pct);
          MotorShoulder.spin(directionType::fwd);
          // MotorShoulder.startRotateTo(SHOULDER_POS_TOP,vex::rotationUnits::deg);

    }
    else if(Controller1.ButtonR2.pressing()
         || Controller2.ButtonR2.pressing()) { // lowest position
          MotorShoulder.setVelocity(SHOULDER_SPEED_DOWN, vex::velocityUnits::pct);
          MotorShoulder.spin(directionType::rev);
          // MotorShoulder.startRotateTo(SHOULDER_POS_BOTTOM,vex::rotationUnits::deg);

    }
    else { // stop
        MotorShoulder.stop();
    }

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Shoulder: ");
    Brain.Screen.print(MotorShoulder.position(rotationUnits::deg));
    Brain.Screen.newLine();

    //Claw Control
    if((Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing()) 
    || (Controller2.ButtonL1.pressing() && Controller2.ButtonL2.pressing())) { // middle position
          MotorClaw.startRotateTo(CLAW_MID,vex::rotationUnits::deg);
    }
    else if(Controller1.ButtonL1.pressing()
         || Controller2.ButtonL1.pressing()) { // closed position
          MotorClaw.spin(directionType::rev);
          // MotorClaw.startRotateTo(CLAW_CLOSED,vex::rotationUnits::deg);

    }
    else if(Controller1.ButtonL2.pressing()
         || Controller2.ButtonL2.pressing()) { // widest position
          MotorClaw.spin(directionType::fwd);
          // MotorClaw.startRotateTo(CLAW_OPEN,vex::rotationUnits::deg);

    }
    else { // stop
        MotorClaw.stop();
    }
    
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Claw: ");
    Brain.Screen.print(MotorClaw.position(rotationUnits::deg));
    Brain.Screen.newLine();

    // holonomic control...

    int axis1 = 0;
    int axis3 = 0;
    int axis4 = 0;

    if (Controller1.ButtonUp.pressing()) {
      axis3 = 50; axis4 = 0; axis1 = 0;
      stabilize_axes_by_gyro(axis1, 0);
    } else if (Controller1.ButtonDown.pressing()) {
      axis3 = -50; axis4 = 0; axis1 = 0;
      stabilize_axes_by_gyro(axis1, 0);
    } else if (Controller1.ButtonLeft.pressing()) {
      axis3 = 0; axis4 = -50; axis1 = 0;
      stabilize_axes_by_gyro(axis1, 0);
    } else if (Controller1.ButtonRight.pressing()) {
      axis3 = 0; axis4 = 50; axis1 = 0;
      stabilize_axes_by_gyro(axis1, 0);
    } else {
      // get the axes values
      axis1 = Controller1.Axis1.position(pct) * SPINSCALE;
      axis3 = Controller1.Axis3.position(pct);
      axis4 = Controller1.Axis4.position(pct);
    }

    // Brain.Screen.setCursor(1, 1);
    // Brain.Screen.print("Rotation:");
    // Brain.Screen.print(InertialSensor.heading(degrees));
    // Brain.Screen.newLine();
    // Brain.Screen.print("Y Before: ");
    // Brain.Screen.print(FORMAT, axis3);

    adjust_axes_for_heading (axis3, axis4);

    // Brain.Screen.newLine();
    // Brain.Screen.print("Y After : ");
    // Brain.Screen.print(FORMAT, axis3);

    double front_left  = 0; double back_left   = 0; double front_right = 0; double back_right  = 0;
    basic_motor_calculation (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
    normalize_motor_power (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
    apply_motor_power (front_left, back_left, front_right, back_right);

    }
}




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
