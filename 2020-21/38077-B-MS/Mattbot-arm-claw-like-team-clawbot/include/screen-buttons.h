void DrawButtons();
void HiliteTouch();
int AutonLocation();

enum { AUTON_DO_NOTHING,
       AUTON_FOREVER_LINE_FOLLOW,
       AUTON_RED_LEFT_1_BALL,
       AUTON_RED_LEFT_2_BALLS,
       AUTON_RED_RIGHT_1_BALL,
       AUTON_RED_RIGHT_2_BALLS,
       AUTON_BLUE_LEFT_1_BALL,
       AUTON_BLUE_LEFT_2_BALLS,
       AUTON_BLUE_RIGHT_1_BALL,
       AUTON_BLUE_RIGHT_2_BALLS
};
     
// which autonomous variation to run
extern int AutonChoice;
