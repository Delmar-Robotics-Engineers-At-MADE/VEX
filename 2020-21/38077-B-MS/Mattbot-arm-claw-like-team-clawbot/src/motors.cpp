#include "vex.h"
#include <algorithm>
#include <cmath>

#include "motors.h"

void normalize_motor_power (int axis1, int axis3, int axis4, double &front_left, double &front_right) {
  // Reference: https://www.robotmesh.com/studio/5be40c90c8f17a1f5796fd35?fbclid=IwAR3g3JMtKeQtWPKUU2bsRnmOdJsWAlkyqhnRw0QpEnHRVIeOMx74JPqXGWE

        //Find the largest possible sum of X and Y
        double max_raw_sum = (double)(std::abs(axis3) + std::abs(axis4));
        
        //Find the largest joystick value
        double max_XYstick_value = (double)(std::max(std::abs(axis3),std::abs(axis4)));
        
        //The largest sum will be scaled down to the largest joystick value, and the others will be
        //scaled by the same amount to preserve directionality
        if (max_raw_sum != 0) {
            front_left  = front_left / max_raw_sum * max_XYstick_value;
            //back_left   = back_left / max_raw_sum * max_XYstick_value;
            front_right = front_right / max_raw_sum * max_XYstick_value;
            //back_right  = back_right / max_raw_sum * max_XYstick_value;
        }
        
        //Now to consider rotation
        //Naively add the rotational axis
        front_left  = front_left  + axis1;
        //back_left   = back_left   + axis1;
        front_right = front_right - axis1;
        //back_right  = back_right  - axis1;
        
        //What is the largest sum, or is 100 larger?
        max_raw_sum = std::max(std::abs(front_left),std::max(std::abs(front_right),100.0));
        
        //Scale everything down by the factor that makes the largest only 100, if it was over
        front_left  = front_left  / max_raw_sum * 100.0;
        //back_left   = back_left   / max_raw_sum * 100.0;
        front_right = front_right / max_raw_sum * 100.0;
        //back_right  = back_right  / max_raw_sum * 100.0;
}

void apply_motor_power (double front_left, double front_right) {
          //Write the manipulated values out to the motors
         front_left_motor.spin(fwd,front_left, velocityUnits::pct);
          // back_left_motor.spin(fwd,back_left,  velocityUnits::pct);
        front_right_motor.spin(fwd,front_right,velocityUnits::pct);
         // back_right_motor.spin(fwd,back_right, velocityUnits::pct);
}

#define PI 3.14159265
#define FORMAT "%.1f" /* 1 decimal place  (0.1) */

// void adjust_axes_for_heading (int &y, int&x) {
//   // reference: https://pdocs.kauailabs.com/navx-mxp/examples/field-oriented-drive/
//   double gyro_degrees = InertialSensor.heading(degrees);
//   float gyro_radians = gyro_degrees * PI/180; 
//   float temp = y * cos(gyro_radians) + x * sin(gyro_radians);
//   x = -y * sin(gyro_radians) + x * cos(gyro_radians);
//   y = temp;
// }

void basic_motor_calculation (int axis1, int axis3, int axis4, double &front_left, double &front_right) {
      //Get the raw sums of the X and Y joystick axes
    front_left  = (double)(axis3 + axis4);
    //back_left   = (double)(axis3 - axis4);
    front_right = (double)(axis3 - axis4);
    //back_right  = (double)(axis3 + axis4);
}

// void stabilize_axes_by_gyro (int &axis1, double targetRotation) {
//       double rotation = InertialSensor.rotation(degrees);
//       double error = rotation - targetRotation;
//       if (error < -GYROTOLERANCE) {axis1 = GYROCORRECT;}
//       else if (error > GYROTOLERANCE) {axis1 = -GYROCORRECT;}
//       else axis1 = 0;
// }
