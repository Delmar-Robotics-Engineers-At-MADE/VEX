#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
/*vex-vision-config:begin*/
signature Vision21__SIG_0 = signature (1, -2929, -2109, -2519, 5019, 7263, 6141, 3, 0);
vision Vision21 = vision (PORT21, 50, Vision21__SIG_0);
/*vex-vision-config:end*/
motor LeftDriveSmart = motor(PORT1, ratio18_1, false);
motor RightDriveSmart = motor(PORT10, ratio18_1, true);
gyro DrivetrainGyro = gyro(Brain.ThreeWirePort.B);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainGyro, 319.19, 320, 40, mm, 1);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Gyro
  wait(200, msec);
  DrivetrainGyro.calibrate();
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the Gyro calibration process to finish
  while (DrivetrainGyro.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}