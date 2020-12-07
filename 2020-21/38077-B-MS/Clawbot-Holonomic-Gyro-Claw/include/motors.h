void normalize_motor_power (int axis1, int axis3, int axis4, double &front_left, double &back_left, double &front_right, double &back_right);
void apply_motor_power (double front_left, double back_left, double front_right, double back_right);
void adjust_axes_for_heading (int &y, int&x);
void basic_motor_calculation (int axis1, int axis3, int axis4, double &front_left, double &back_left, double &front_right, double &back_right);
void stabilize_axes_by_gyro (int &axis1);

#define SPINSCALE 0.5
#define GYROCORRECT 20
#define GYROTOLERANCE 5
