#include "vex.h"
#include <algorithm>
#include <cmath>
// #include <robot-config.h> ... should be included by vex.h
#include "screen-buttons.h"
#include "autonomous.h"
#include "motors.h"

using namespace vex;

// flag so pre_auton runs before tele-op when off competition switch
bool pre_auton_done = false;

int line_tracker_status (int threshold, line trackerA, line trackerB, line trackerC) {
  int result = LINELOST;
  int a = trackerA.reflectivity(); int b = trackerB.reflectivity(); int c = trackerC.reflectivity();
  bool aON = (a >= threshold); bool bON = (b >= threshold); bool cON = (c >= threshold);
  if (aON && bON && cON) {result = LINEALLON;}
  else if (aON && bON) {result = LINETHICKOFFTORIGHT;}
  else if (bON && cON) {result = LINETHICKOFFTOLEFT;}
  else if (aON) {result = LINEOFFTORIGHT;}
  else if (bON) {result = LINECENTERED;}
  else if (cON) {result = LINEOFFTOLEFT;}
  return result;
}

void basic_line_follow (int &axis1, int &axis3, int &axis4, int threshold, 
                        line trackerA, line trackerB, line trackerC, 
                        int speed, int correctSpeed, double angle) {

        int lineStatus = line_tracker_status(threshold, trackerA, trackerB, trackerC);
        if (lineStatus == LINECENTERED || lineStatus == LINEALLON) {
          stabilize_axes_by_gyro (axis1, angle);
          axis3 = speed; axis4 = 0;
        } else if (lineStatus == LINEOFFTOLEFT || lineStatus == LINETHICKOFFTOLEFT) {
          // veer right to get centered again
          axis1 = 0; axis3 = speed; axis4 = correctSpeed;
        } else if (lineStatus == LINEOFFTORIGHT || lineStatus == LINETHICKOFFTORIGHT) {
          // veer left to get centered again
          axis1 = 0; axis3 = speed; axis4 = -correctSpeed;
        } else if (lineStatus == LINELOST) {
          // wait a little and maybe line will reappear
          wait (0.1, sec);
          if (line_tracker_status(threshold, trackerA, trackerB, trackerC) == LINELOST) { 
            axis1 = 0; axis3 = 0; axis4 = 0;
          }
        }
        // make field-relative so robot can follow line sideways
        adjust_axes_for_heading (axis3, axis4);
}


// ******************************************************* Autonomous *******************************************************************

void autonomous(void) {

  while (!pre_auton_done) {
    vex::task::sleep(100);
  }

  // while (true) {
  //   // Clear the screen and set the cursor to top left corner on each loop
  //   Brain.Screen.clearScreen();
  //   Brain.Screen.setCursor(1, 1);
  //   Brain.Screen.print("Reflectivity A: ");
  //   Brain.Screen.print(FORMAT, static_cast<float>(LineTrackerA.reflectivity()));
  //   Brain.Screen.newLine();
  //   Brain.Screen.print("Reflectivity B: ");
  //   Brain.Screen.print(FORMAT, static_cast<float>(LineTrackerB.reflectivity()));
  //   Brain.Screen.newLine();
  //   Brain.Screen.print("Reflectivity C: ");
  //   Brain.Screen.print(FORMAT, static_cast<float>(LineTrackerC.reflectivity()));
  //   Brain.Screen.newLine();
  //   // A brief delay to allow text to be printed without distortion or tearing
  //   wait(0.05, seconds);
  // wait(5, msec);
  // }

// while (true) {
//   Brain.Screen.clearScreen();
//   Brain.Screen.setCursor(1, 1);
//   Brain.Screen.print("Line status: ");
//   Brain.Screen.print(line_tracker_status());
//   wait (50, msec);
// }

  // do the appropriate maneuver, depending on screen button selection

  double front_left  = 0; double back_left   = 0; double front_right = 0; double back_right  = 0;
  InertialSensor.setRotation(0, degrees); // reset heading to 0
  int axis1 = 0; int axis3 = 0; int axis4 = 0;

  switch (AutonChoice) {
    case AUTON_DO_NOTHING: break;

    case AUTON_FOREVER_LINE_FOLLOW:
      {
      // drive straight line until we lose line

      bool giveUp = false;
      bool lookingForLine = false;

      while (!giveUp) {

        // figure out what to do

        int lineStatus = line_tracker_status(LINETHRESHOLD, LineTrackerA, LineTrackerB, LineTrackerC);
        if (lookingForLine && (lineStatus == LINECENTERED || lineStatus == LINEALLON)) { // found line again
          InertialSensor.setRotation(0, degrees); // reset heading to 0
          lookingForLine = false;
        } else if (lineStatus == LINECENTERED || lineStatus == LINEALLON) {
          // basicaly go straight, but correct for gyro
          stabilize_axes_by_gyro (axis1, 0);
          axis3 = LINESPEED; axis4 = 0;
        } else if (lineStatus == LINEOFFTOLEFT || lineStatus == LINETHICKOFFTOLEFT) {
          // veer right to get centered again
          axis1 = 0; axis3 = LINESPEED; axis4 = LINECORRECT;
        } else if (lineStatus == LINEOFFTORIGHT || lineStatus == LINETHICKOFFTORIGHT) {
          // veer left to get centered again
          axis1 = 0; axis3 = LINESPEED; axis4 = -LINECORRECT;
        } else if (lookingForLine) {
          if (InertialSensor.rotation(degrees) >= 300) { // swiveled for too long
            giveUp = true;
          } else {
            axis1 = GYROCORRECT; axis3 = 0; axis4 = LINESPEED / 2; // swivel for a while until find line
          }
        } else if (lineStatus == LINELOST) {
          // wait a little and maybe line will reappear
          wait (0.1, sec);
          if (line_tracker_status(LINETHRESHOLD, LineTrackerA, LineTrackerB, LineTrackerC) == LINELOST) { // still lost, try once to find it again by swiveling
            InertialSensor.setRotation(0, degrees); // will look for line until swiveled too far
            lookingForLine = true;
          }
        }

        // do it
        basic_motor_calculation (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
        normalize_motor_power (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
        apply_motor_power (front_left, back_left, front_right, back_right);

      } // while not give up

      // give up
      apply_motor_power (0, 0, 0, 0);

      while (true) {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Line status: ");
        Brain.Screen.print(line_tracker_status(LINETHRESHOLD, LineTrackerA, LineTrackerB, LineTrackerC));
        Brain.Screen.newLine();
        Brain.Screen.print("rotation: ");
        Brain.Screen.print(InertialSensor.rotation(degrees));
        wait (50, msec);
      }
      } // scope of case, which has its own variables

      break; // end of forever line follow auto

    case AUTON_BLUE_LEFT_2_BALLS:

        {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Running Blue Left 2 balls");

        // collect ball for travelling
        MotorClaw.startRotateTo(100,vex::rotationUnits::deg);

        // reverse to line, until middle side sensor sees it
        int lineStatus = LINELOST;
        while (lineStatus != LINEOFFTOLEFT) {
          // basicaly go straight, but correct for gyro
          stabilize_axes_by_gyro (axis1, 0);
          axis3 = -LINESPEED/2; axis4 = 0;
          basic_motor_calculation (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          normalize_motor_power (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          apply_motor_power (front_left, back_left, front_right, back_right);
          lineStatus = line_tracker_status(LINETHRESHOLD, LineTrackerD, LineTrackerE, LineTrackerF);
          // Brain.Screen.setCursor(2, 1);
          // Brain.Screen.print(lineStatus); 
        }
        apply_motor_power (0, 0, 0, 0);

        // raise arm
        MotorShoulder.setVelocity(75, vex::velocityUnits::pct);
        MotorShoulder.startRotateTo(840,vex::rotationUnits::deg);
        wait (1000, msec);

        // move down line to next goal, when front trackers see line
        InertialSensor.setHeading(275, degrees);  // drive down line sideways
        InertialSensor.setRotation(90, degrees);  // drive down line sideways
        lineStatus = LINELOST;
        while (lineStatus == LINELOST) {
          basic_line_follow (axis1, axis3, axis4, LINETHRESHOLD, 
                        LineTrackerD, LineTrackerE, LineTrackerF, 
                        -LINESPEED, LINECORRECT, 90);
          basic_motor_calculation (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          normalize_motor_power (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          apply_motor_power (front_left, back_left, front_right, back_right);
          lineStatus = line_tracker_status(LINETHRESHOLD, LineTrackerA, LineTrackerB, LineTrackerC);
          // Brain.Screen.setCursor(2, 1);
          // Brain.Screen.clearLine();
          // Brain.Screen.print(axis1);         
          // Brain.Screen.print(" ");         
          // Brain.Screen.print(axis3);         
          // Brain.Screen.print(" ");         
          // Brain.Screen.print(axis4);         
          // Brain.Screen.setCursor(3, 1);
          // Brain.Screen.clearLine();
          // Brain.Screen.print(InertialSensor.heading(degrees));         
          // Brain.Screen.setCursor(4, 1);
          // Brain.Screen.clearLine();
          // Brain.Screen.print(lineStatus);         
        }
        apply_motor_power (0, 0, 0, 0);
        wait (1000, msec);

        // scoot forward a little and score ball
        int targetRotationDegrees = 150;
        front_left_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        front_right_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        back_left_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        back_right_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        while (front_left_motor.isSpinning()) {task::sleep(100);}
        MotorClaw.startRotateTo(-400,vex::rotationUnits::deg);


        }
        break;      

  }

}
