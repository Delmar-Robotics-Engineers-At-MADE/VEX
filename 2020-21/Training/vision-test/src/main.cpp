/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Fri Sep 27 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*    This project will detect 3 different colored objects and display        */
/*    when each object is found on the V5 Brain's screen.                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision5              vision        5
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

event checkRed = event();
event checkBlue = event();

void hasBlueCallback() {
  Brain.Screen.setFont(mono40);
  Brain.Screen.clearLine(1, black);
  Brain.Screen.setCursor(Brain.Screen.row(), 1);
  Brain.Screen.setCursor(1, 1);
  Vision8.takeSnapshot(Vision8__CHGUP_BALL_BLUE);
  if (Vision8.objectCount > 0) {
    Brain.Screen.print("Blue Object Found");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print(Vision8.largestObject.centerX);
    Brain.Screen.print(" ");
    Brain.Screen.print(Vision8.largestObject.centerY);
    Brain.Screen.print(" ");
    Brain.Screen.print(Vision8.largestObject.width * Vision8.largestObject.height);
    Brain.Screen.print("          ");
  } else {
    Brain.Screen.print("No Blue Object");
  }
}

void hasRedCallback() {
  Brain.Screen.setFont(mono40);
  Brain.Screen.clearLine(3, black);
  Brain.Screen.setCursor(Brain.Screen.row(), 1);
  Brain.Screen.setCursor(3, 1);
  Vision8.takeSnapshot(Vision8__CHGUP_BALL_RED);
  if (Vision8.objectCount > 0) {
    Brain.Screen.print("Red Object Found");
  } else {
    Brain.Screen.print("No Red Object");
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  checkBlue(hasBlueCallback);
  checkRed(hasRedCallback);

  while (true) {
    checkBlue.broadcastAndWait();
    checkRed.broadcastAndWait();
    wait(1, seconds);
  }
}
