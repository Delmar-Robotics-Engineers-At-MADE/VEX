#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
//motor back_right_motor = motor(PORT2, ratio18_1, true); // 12 for scratchbot
//motor back_left_motor = motor(PORT9, ratio18_1, false); // 11 for scratchbot
motor front_right_motor = motor(PORT10, ratio18_1, true); // 2 for scratchbot
motor front_left_motor = motor(PORT1, ratio18_1, false); // 1 for scratchbot
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
// inertial InertialSensor = inertial(PORT16);
// line LineTrackerA = line(Brain.ThreeWirePort.A);
// line LineTrackerB = line(Brain.ThreeWirePort.B);
// line LineTrackerC = line(Brain.ThreeWirePort.C);
// line LineTrackerD = line(Brain.ThreeWirePort.D);
// line LineTrackerE = line(Brain.ThreeWirePort.E);
// line LineTrackerF = line(Brain.ThreeWirePort.F);
motor MotorClaw = motor(PORT3, ratio18_1, false);
motor MotorShoulder = motor(PORT8, ratio18_1, false);

/*vex-vision-config:begin*/
signature Vision8__CHGUP_BALL_BLUE = 
  signature(1,-3347,-2737,-3042,9863,12407,11134,3.0,0);

signature Vision8__CHGUP_BALL_RED =
  signature(2,4949,6023,5486,-1,1401,700,2.5,0);

signature Vision8__SIG_3 = signature(4, 0, 0, 0, 0, 0, 0, 2.5, 0);
signature Vision8__SIG_4 = signature(4, 0, 0, 0, 0, 0, 0, 2.5, 0);
signature Vision8__SIG_5 = signature(5, 0, 0, 0, 0, 0, 0, 2.5, 0);
signature Vision8__SIG_6 = signature(6, 0, 0, 0, 0, 0, 0, 2.5, 0);
signature Vision8__SIG_7 = signature(7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vision Vision8 =
    vision(PORT21, 50, Vision8__CHGUP_BALL_BLUE, Vision8__CHGUP_BALL_RED, Vision8__SIG_3,
           Vision8__SIG_4, Vision8__SIG_5, Vision8__SIG_6, Vision8__SIG_7);
/*vex-vision-config:end*/

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