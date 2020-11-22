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

void apply_motor_power (double &front_left, double &back_left, double &front_right, double &back_right) {
          //Write the manipulated values out to the motors
         front_left_motor.spin(fwd,front_left, velocityUnits::pct);
          back_left_motor.spin(fwd,back_left,  velocityUnits::pct);
        front_right_motor.spin(fwd,front_right,velocityUnits::pct);
         back_right_motor.spin(fwd,back_right, velocityUnits::pct);
}

#define PI 3.14159265
void adjust_axes_for_heading (double &x, double&y) {
  // reference: https://pdocs.kauailabs.com/navx-mxp/examples/field-oriented-drive/
  double gyro_degrees = InertialSensor.rotation(degrees);
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
    // get the axes values
    double axis1 = Controller1.Axis1.position(pct);
    double axis3 = Controller1.Axis3.position(pct);
    double axis4 = Controller1.Axis4.position(pct);

    //Get the raw sums of the X and Y joystick axes
    double front_left  = (double)(axis3 + axis4);
    double back_left   = (double)(axis3 - axis4);
    double front_right = (double)(axis3 - axis4);
    double back_right  = (double)(axis3 + axis4);

    normalize_motor_power (axis1, axis3, axis4, front_left, back_left, front_right, back_right);
    apply_motor_power (front_left, back_left, front_right, back_right);

    }
}
