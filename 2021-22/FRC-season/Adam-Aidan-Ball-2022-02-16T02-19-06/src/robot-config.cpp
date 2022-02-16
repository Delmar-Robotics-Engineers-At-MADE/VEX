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
motor LeftMotor = motor(PORT1, ratio18_1, true);
motor RightMotor = motor(PORT10, ratio18_1, false);
gyro GyroB = gyro(Brain.ThreeWirePort.B);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}