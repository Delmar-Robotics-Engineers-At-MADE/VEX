using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern signature Vision8__CHGUP_BALL_BLUE;
extern signature Vision8__CHGUP_BALL_RED;
extern signature Vision8__SIG_3;
extern signature Vision8__SIG_4;
extern signature Vision8__SIG_5;
extern signature Vision8__SIG_6;
extern signature Vision8__SIG_7;
extern vision Vision8;

extern motor MotorShoulder;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);