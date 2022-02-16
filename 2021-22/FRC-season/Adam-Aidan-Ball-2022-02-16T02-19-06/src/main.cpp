/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\VEX                                              */
/*    Created:      Thu Feb 10 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Vision21             vision        21              
// LeftMotor            motor         1               
// RightMotor           motor         10              
// GyroB                gyro          B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
int Brain_precision = 0, Console_precision = 0, Controller1_precision = 0, Vision21_objectIndex = 0;
float myVariable, error;


const char* printToBrain_numberFormat() {
  // look at the current precision setting to find the format string
  switch(Brain_precision){
    case 0:  return "%.0f"; // 0 decimal places (1)
    case 1:  return "%.1f"; // 1 decimal place  (0.1)
    case 2:  return "%.2f"; // 2 decimal places (0.01)
    case 3:  return "%.3f"; // 3 decimal places (0.001)
    default: return "%f"; // use the print system default for everthing else
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  float const P2 = 0.9;
  float const P = 0.4;
  float const target_height = 100;
  float error2 = 0;
  float forward_speed = 0;
  float diff_speed = 0;
  while (true)
  {
    Vision21.takeSnapshot(Vision21__SIG_0);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.clearLine(1);
    Brain.Screen.setCursor(Brain.Screen.row(), 1);
    Brain.Screen.print(printToBrain_numberFormat(), static_cast<float>(Vision21.objectCount));
    int biggest_object = 0;
    for (int i = 0; i < Vision21.objectCount; i++)
    {
      if (Vision21.objects[i].height > (Vision21.objects[biggest_object].height))
      {
        biggest_object = Vision21.objects[i].height;
      }
      Vision21_objectIndex = static_cast<int>(1.0) - 1;
      error = Vision21.objects[Vision21_objectIndex].centerX - 180.0;
      error2 = biggest_object - target_height;
      forward_speed = P2 * error2;
      diff_speed = P * error;
      LeftMotor.setVelocity(forward_speed + diff_speed, percent);
      RightMotor.setVelocity(forward_speed - diff_speed, percent);
      RightMotor.spin(forward);
      LeftMotor.spin(forward);
    }
    
    
    wait(5, msec);
  }
  
}
