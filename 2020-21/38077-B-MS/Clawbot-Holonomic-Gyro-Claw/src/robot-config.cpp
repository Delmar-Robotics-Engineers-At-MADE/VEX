#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor back_right_motor = motor(PORT2, ratio18_1, true); // 12 for scratchbot
motor back_left_motor = motor(PORT9, ratio18_1, false); // 11 for scratchbot
motor front_right_motor = motor(PORT1, ratio18_1, true); // 2 for scratchbot
motor front_left_motor = motor(PORT10, ratio18_1, false); // 1 for scratchbot
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
inertial InertialSensor = inertial(PORT16);
line LineTrackerA = line(Brain.ThreeWirePort.A);
line LineTrackerB = line(Brain.ThreeWirePort.B);
line LineTrackerC = line(Brain.ThreeWirePort.C);
motor MotorClaw = motor(PORT5, ratio18_1, false);
motor MotorShoulder = motor(PORT6, ratio18_1, false);

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