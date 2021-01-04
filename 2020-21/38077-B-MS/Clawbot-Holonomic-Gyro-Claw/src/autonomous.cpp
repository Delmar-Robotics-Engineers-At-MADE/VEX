#include "vex.h"
#include <algorithm>
#include <cmath>
// #include <robot-config.h> ... should be included by vex.h
#include "screen-buttons.h"
#include "autonomous.h"
#include "motors.h"

using namespace vex;

// global instance of autonomous class
Autonomous autorunner;

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

void Autonomous::move_claw_off_wheels (void) {
  MotorShoulder.setVelocity(20, vex::velocityUnits::pct);
  MotorShoulder.startRotateTo(110,vex::rotationUnits::deg);
  while (MotorShoulder.isSpinning()) {task::sleep(100);}
}

void Autonomous::grab_ball_for_travelling (void) {
        MotorClaw.startRotateTo(CLAW_CLOSED,vex::rotationUnits::deg);
        while (MotorClaw.isSpinning()) {task::sleep(100);}
}

void Autonomous::raise_arm_a_little_for_travelling (void) {
        MotorShoulder.setVelocity(SHOULDER_SPEED_UP, vex::velocityUnits::pct);
        MotorShoulder.startRotateTo(SHOULDER_POS_MID,vex::rotationUnits::deg);
}

void Autonomous::reverse_to_line_until_middle_side_sensor_sees_it (void) {
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
}

void Autonomous::raise_arm_to_top_for_travelling (void) {
        MotorShoulder.setVelocity(SHOULDER_SPEED_UP, vex::velocityUnits::pct);
        MotorShoulder.startRotateTo(SHOULDER_POS_TOP,vex::rotationUnits::deg);
}

void Autonomous::move_down_line_to_next_goal_when_front_trackers_see_line (void) {
        InertialSensor.setHeading(275, degrees);  // drive down line sideways
        InertialSensor.setRotation(90, degrees);  // drive down line sideways
        int lineStatus = LINELOST;
        while (lineStatus == LINELOST) {
          basic_line_follow (axis1, axis3, axis4, LINETHRESHOLD, 
                        LineTrackerD, LineTrackerE, LineTrackerF, 
                        -LINESPEED, LINECORRECT, 90);
          basic_motor_calculation (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          normalize_motor_power (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          apply_motor_power (front_left, back_left, front_right, back_right);
          lineStatus = line_tracker_status(LINETHRESHOLD, LineTrackerA, LineTrackerB, LineTrackerC);     
        }
        apply_motor_power (0, 0, 0, 0);
}

void Autonomous::scoot_forward_a_little_and_score_ball  (void) {
        int targetRotationDegrees = 150;
        front_left_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        front_right_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        back_left_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        back_right_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        while (front_left_motor.isSpinning()) {task::sleep(100);}
        MotorClaw.startRotateTo(CLAW_OPEN,vex::rotationUnits::deg);
        wait (1000, msec);
}

void Autonomous::swivel_back_to_line_until_middle_side_sensor_sees_it (void) {
        InertialSensor.resetHeading();
        InertialSensor.resetRotation();
        int lineStatus = LINELOST;
        while (lineStatus != LINEOFFTOLEFT) {
          // basicaly go straight, but correct for gyro
          stabilize_axes_by_gyro (axis1, 0);
          axis3 = -LINESPEED/2; axis4 = LINESPEED/2;
          basic_motor_calculation (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          normalize_motor_power (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          apply_motor_power (front_left, back_left, front_right, back_right);
          lineStatus = line_tracker_status(LINETHRESHOLD, LineTrackerD, LineTrackerE, LineTrackerF);
          // Brain.Screen.setCursor(2, 1);
          // Brain.Screen.print(lineStatus); 
        }
        apply_motor_power (0, 0, 0, 0);
}

void Autonomous::rotate_right_until_front_sensors_are_on_line (void) {
        int lineStatus = LINELOST;
        while (lineStatus == LINELOST) {
          axis1 = GYROCORRECT; axis3 = 0; axis4 = 0;
          basic_motor_calculation (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          normalize_motor_power (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          apply_motor_power (front_left, back_left, front_right, back_right);
          lineStatus = line_tracker_status(LINETHRESHOLD, LineTrackerA, LineTrackerB, LineTrackerC);
        }
        apply_motor_power(0, 0, 0, 0);
}

void Autonomous::lower_arm (void) {
        MotorShoulder.setVelocity(SHOULDER_SPEED_DOWN, vex::velocityUnits::pct);
        MotorShoulder.startRotateTo(SHOULDER_POS_BOTTOM,vex::rotationUnits::deg);
        while (MotorShoulder.isSpinning()) {task::sleep(100);}
}

void Autonomous::drive_forward_until_ball_is_off_to_the_left (void) {
        // abort if we drive too far
        InertialSensor.resetHeading();
        InertialSensor.resetRotation();
        front_left_motor.resetPosition();
        bool ballPositioned = false; bool abort = false;
        int abortDistance = 1600;
        while (!ballPositioned && !abort) {
          abort = (front_left_motor.position(rotationUnits::deg) >= abortDistance);
          basic_line_follow (axis1, axis3, axis4, LINETHRESHOLD, 
                  LineTrackerA, LineTrackerB, LineTrackerC, 
                  LINESPEED, LINECORRECT, 0);
          basic_motor_calculation (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          normalize_motor_power (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          apply_motor_power (front_left, back_left, front_right, back_right);
          Vision8.takeSnapshot(Vision8__CHGUP_BALL_BLUE);
          ballPositioned = (Vision8.objectCount > 0 && Vision8.largestObject.centerX < 15);
          Brain.Screen.setCursor(1, 1);
          Brain.Screen.clearLine();
          Brain.Screen.print(front_left_motor.position(rotationUnits::deg));
        }
        apply_motor_power(0, 0, 0, 0);
        if (abort) {while(true){Brain.Screen.setCursor(2, 1); Brain.Screen.setPenColor(color::red); Brain.Screen.print("aborted due to distance 1");}}
}

void Autonomous::rotate_toward_ball (void) {
        while(InertialSensor.rotation(degrees) >= -25) {
          axis1 = -GYROCORRECT; axis3 = 0; axis4 = 0;
          basic_motor_calculation (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          normalize_motor_power (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          apply_motor_power (front_left, back_left, front_right, back_right);
        }
        apply_motor_power(0, 0, 0, 0);
}

void Autonomous::drive_forward_until_ball_is_in_clutches (void) {
        InertialSensor.resetHeading();
        InertialSensor.resetRotation();
        front_left_motor.resetPosition();
        bool ballPositioned = false; bool abort = false;
        int abortDistance = 1600;
        while (!ballPositioned && !abort) {
          abort = front_left_motor.position(rotationUnits::deg) >= abortDistance;
          basic_line_follow (axis1, axis3, axis4, LINETHRESHOLD, 
                  LineTrackerA, LineTrackerB, LineTrackerC, 
                  LINESPEED, LINECORRECT, 0);
          basic_motor_calculation (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          normalize_motor_power (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
          apply_motor_power (front_left, back_left, front_right, back_right);
          Vision8.takeSnapshot(Vision8__CHGUP_BALL_BLUE);
          ballPositioned = (Vision8.objectCount > 0 && Vision8.largestObject.centerY < 215);
        }
        apply_motor_power(0, 0, 0, 0);
        if (abort) {while(true){Brain.Screen.setCursor(2, 1); Brain.Screen.setPenColor(color::red); Brain.Screen.print("aborted due to distance 2");}}
}

void Autonomous::scoot_forward_a_little_more_and_collect_ball (void) {
        int targetRotationDegrees = 240;
        front_left_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        front_right_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        back_left_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        back_right_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        while (front_left_motor.isSpinning()) {task::sleep(100);}
        MotorClaw.startRotateTo(CLAW_CLOSED,vex::rotationUnits::deg);
        while (MotorClaw.isSpinning()) {task::sleep(100);}
}

void Autonomous::back_up_and_raise_arm(void) {
        int targetRotationDegrees = -230;
        front_left_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        front_right_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        back_left_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        back_right_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        while (front_left_motor.isSpinning()) {task::sleep(100);}
        MotorShoulder.setVelocity(SHOULDER_SPEED_UP, vex::velocityUnits::pct);
        MotorShoulder.startRotateTo(SHOULDER_POS_TOP,vex::rotationUnits::deg);
        while (MotorShoulder.isSpinning()) {task::sleep(100);}
}

void Autonomous::scoot_forward_score_ball (void) {
        int targetRotationDegrees = 410;
        front_left_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        front_right_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        back_left_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        back_right_motor.startSpinFor(targetRotationDegrees, rotationUnits::deg, LINESPEED, velocityUnits::pct);
        while (front_left_motor.isSpinning()) {task::sleep(100);}
        MotorClaw.startRotateTo(CLAW_OPEN,vex::rotationUnits::deg);
        while (MotorClaw.isSpinning()) {task::sleep(100);}
}

// constructor
Autonomous::Autonomous(void) {
  front_left  = 0; back_left   = 0; front_right = 0; back_right  = 0;
  axis1 = 0; axis3 = 0; axis4 = 0;
}

// ******************************************************* Autonomous *******************************************************************

void Autonomous::runAutonomous(void) {

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

  InertialSensor.setRotation(0, degrees); // reset heading to 0

  switch (AutonChoice) {
    case AUTON_DO_NOTHING: 
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("Do-nothing auto");
      move_claw_off_wheels();
      break;

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

    case AUTON_BLUE_LEFT_1_BALL:
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Variant not implemented");

        {
        move_claw_off_wheels();

        // grab ball for travelling
        MotorClaw.startRotateTo(CLAW_CLOSED,vex::rotationUnits::deg);
        while (MotorClaw.isSpinning()) {task::sleep(100);}

        }

        break;

    case AUTON_BLUE_LEFT_2_BALLS:

        {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Running Blue Left 2 balls");

        move_claw_off_wheels();

        grab_ball_for_travelling();

        raise_arm_a_little_for_travelling();

        reverse_to_line_until_middle_side_sensor_sees_it ();

        raise_arm_to_top_for_travelling();

        move_down_line_to_next_goal_when_front_trackers_see_line ();

        scoot_forward_a_little_and_score_ball ();

        swivel_back_to_line_until_middle_side_sensor_sees_it ();

        rotate_right_until_front_sensors_are_on_line ();

        lower_arm();

        drive_forward_until_ball_is_off_to_the_left ();

        rotate_toward_ball();

        drive_forward_until_ball_is_in_clutches();

        scoot_forward_a_little_more_and_collect_ball();

        back_up_and_raise_arm();

        scoot_forward_score_ball();

        }
        break;      

    default:
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("Variant not implemented");

  }

}


void autonomous(void) {
  autorunner.runAutonomous();
}