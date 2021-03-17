void normalize_motor_power (int axis1, int axis3, int axis4, double &front_left, double &back_left, double &front_right, double &back_right);
void apply_motor_power (double front_left, double back_left, double front_right, double back_right);
void adjust_axes_for_heading (int &y, int&x);
void basic_motor_calculation (int axis1, int axis3, int axis4, double &front_left, double &back_left, double &front_right, double &back_right);
void stabilize_axes_by_gyro (int &axis1, double targetRotation);

#define SPINSCALE 0.5
#define GYROCORRECT 20
#define GYROTOLERANCE 5

#define CLAW_CLOSED 190
#define CLAW_OPEN 20
#define CLAW_MID 60

#define SHOULDER_SPEED_UP 40
#define SHOULDER_SPEED_DOWN 20
#define SHOULDER_POS_BOTTOM 110
#define SHOULDER_POS_TOP 900
#define SHOULDER_POS_MID 300