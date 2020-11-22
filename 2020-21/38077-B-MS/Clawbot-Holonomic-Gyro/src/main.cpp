// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// back_right_motor     motor         12              
// back_left_motor      motor         11              
// front_right_motor    motor         2               
// front_left_motor     motor         1               
// Controller1          controller                    
// InertialSensor       inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// back_right_motor     motor         12              
// back_left_motor      motor         11              
// front_right_motor    motor         2               
// front_left_motor     motor         10              
// Controller1          controller                    
// InertialSensor       inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// back_right_motor     motor         12              
// back_left_motor      motor         11              
// front_right_motor    motor         1               
// front_left_motor     motor         10              
// Controller1          controller                    
// InertialSensor       inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// back_right_motor     motor         12              
// back_left_motor      motor         9               
// front_right_motor    motor         1               
// front_left_motor     motor         10              
// Controller1          controller                    
// InertialSensor       inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// back_right_motor     motor         2               
// back_left_motor      motor         9               
// front_right_motor    motor         1               
// front_left_motor     motor         10              
// Controller1          controller                    
// InertialSensor       inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\msnyd                                            */
/*    Created:      Tue Oct 06 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// back_right_motor     motor         2               
// back_left_motor      motor         9               
// front_right_motor    motor         1               
// front_left_motor     motor         10              
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <algorithm>
#include <cmath>

using namespace vex;

void normalize_motor_power (double axis1, double axis3, double axis4, double &front_left, double &back_left, double &front_right, double &back_right) {
  // Reference: https://www.robotmesh.com/studio/5be40c90c8f17a1f5796fd35?fbclid=IwAR3g3JMtKeQtWPKUU2bsRnmOdJsWAlkyqhnRw0QpEnHRVIeOMx74JPqXGWE

        //Find the largest possible sum of X and Y
        double max_raw_sum = (double)(abs(axis3) + abs(axis4));
        
        //Find the largest joystick value
        double max_XYstick_value = (double)(std::max(abs(axis3),abs(axis4)));
        
        //The largest sum will be scaled down to the largest joystick value, and the others will be
        //scaled by the same amount to preserve directionality
        if (max_raw_sum != 0) {
            front_left  = front_left / max_raw_sum * max_XYstick_value;
            back_left   = back_left / max_raw_sum * max_XYstick_value;
            front_right = front_right / max_raw_sum * max_XYstick_value;
            back_right  = back_right / max_raw_sum * max_XYstick_value;
        }
        
        //Now to consider rotation
        //Naively add the rotational axis
        front_left  = front_left  + axis1;
        back_left   = back_left   + axis1;
        front_right = front_right - axis1;
        back_right  = back_right  - axis1;
        
        //What is the largest sum, or is 100 larger?
        max_raw_sum = std::max(std::abs(front_left),std::max(std::abs(back_left),std::max(std::abs(front_right),std::max(std::abs(back_right),100.0))));
        
        //Scale everything down by the factor that makes the largest only 100, if it was over
        front_left  = front_left  / max_raw_sum * 100.0;
        back_left   = back_left   / max_raw_sum * 100.0;
        front_right = front_right / max_raw_sum * 100.0;
        back_right  = back_right  / max_raw_sum * 100.0;
}

void apply_motor_power (double front_left, double back_left, double front_right, double back_right) {
          //Write the manipulated values out to the motors
         front_left_motor.spin(fwd,front_left, velocityUnits::pct);
          back_left_motor.spin(fwd,back_left,  velocityUnits::pct);
        front_right_motor.spin(fwd,front_right,velocityUnits::pct);
         back_right_motor.spin(fwd,back_right, velocityUnits::pct);
}

#define PI 3.14159265
#define FORMAT "%.1f" /* 1 decimal place  (0.1) */
#define SPINSCALE 0.5

void adjust_axes_for_heading (double &y, double&x) {
  // reference: https://pdocs.kauailabs.com/navx-mxp/examples/field-oriented-drive/
  double gyro_degrees = InertialSensor.heading(degrees);
  float gyro_radians = gyro_degrees * PI/180; 
  float temp = y * cos(gyro_radians) + x * sin(gyro_radians);
  x = -y * sin(gyro_radians) + x * cos(gyro_radians);
  y = temp;
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();  

  // calibrate gyro
  Brain.Screen.print("Calibrating!");
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(50, 50, 400, 100);
  InertialSensor.startCalibration();
  while (InertialSensor.isCalibrating()) { task::sleep(50); }
  wait(3.0, seconds);
  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawCircle(250, 125, 75);
  Brain.Screen.setFillColor(transparent);

  while(true) {

    if (Controller1.ButtonB.pressing()) {  // reset 0 heading to current heading
      InertialSensor.setHeading(0, degrees);
    }

    double axis1 = 0;
    double axis3 = 0;
    double axis4 = 0;

    if (Controller1.ButtonUp.pressing()) {
      axis3 = 50; axis4 = 0; axis1 = 0;
    } else if (Controller1.ButtonDown.pressing()) {
       axis3 = -50; axis4 = 0; axis1 = 0;
    } else if (Controller1.ButtonLeft.pressing()) {
       axis3 = 0; axis4 = -50; axis1 = 0;
    } else if (Controller1.ButtonRight.pressing()) {
       axis3 = 0; axis4 = 50; axis1 = 0;
    } else {
      // get the axes values
      axis1 = Controller1.Axis1.position(pct) * SPINSCALE;
      axis3 = Controller1.Axis3.position(pct);
      axis4 = Controller1.Axis4.position(pct);
    }

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Rotation:");
    Brain.Screen.print(FORMAT, InertialSensor.heading(degrees));
    Brain.Screen.newLine();
    Brain.Screen.print("Y Before: ");
    Brain.Screen.print(FORMAT, axis3);

    adjust_axes_for_heading (axis3, axis4);

    Brain.Screen.newLine();
    Brain.Screen.print("Y After : ");
    Brain.Screen.print(FORMAT, axis3);

    //Get the raw sums of the X and Y joystick axes
    double front_left  = (double)(axis3 + axis4);
    double back_left   = (double)(axis3 - axis4);
    double front_right = (double)(axis3 - axis4);
    double back_right  = (double)(axis3 + axis4);

    normalize_motor_power (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
    apply_motor_power (front_left, back_left, front_right, back_right);

    }
}
