using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
// extern motor back_right_motor;
// extern motor back_left_motor;
extern motor front_right_motor;
extern motor front_left_motor;
extern controller Controller1;
extern controller Controller2;
// extern inertial InertialSensor;
// extern line LineTrackerA;
// extern line LineTrackerB;
// extern line LineTrackerC;
// extern line LineTrackerD;
// extern line LineTrackerE;
// extern line LineTrackerF;
extern motor MotorClaw;
extern motor MotorShoulder;

extern signature Vision8__CHGUP_BALL_BLUE;
extern signature Vision8__CHGUP_BALL_RED;
extern signature Vision8__SIG_3;
extern signature Vision8__SIG_4;
extern signature Vision8__SIG_5;
extern signature Vision8__SIG_6;
extern signature Vision8__SIG_7;
extern vision Vision8;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );