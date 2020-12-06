#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
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
    vision(PORT8, 50, Vision8__CHGUP_BALL_BLUE, Vision8__CHGUP_BALL_RED, Vision8__SIG_3,
           Vision8__SIG_4, Vision8__SIG_5, Vision8__SIG_6, Vision8__SIG_7);
/*vex-vision-config:end*/

// VEXcode generated functions

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}