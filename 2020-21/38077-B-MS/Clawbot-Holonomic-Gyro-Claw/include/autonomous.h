#define LINESPEED 30
#define LINECORRECT 10
#define LINETHRESHOLD 9

#define LINELOST 0
#define LINECENTERED 1
#define LINEOFFTOLEFT 2
#define LINEOFFTORIGHT 3
#define LINETHICKOFFTOLEFT 4
#define LINETHICKOFFTORIGHT 5
#define LINEALLON 6

// flag so pre_auton runs before tele-op when off competition switch
extern bool pre_auton_done;

class Autonomous {
private:
  int axis1, axis3, axis4; 
  double front_left, back_left, front_right, back_right;
public:
  Autonomous(void);
  void runAutonomous(void);
  void move_claw_off_wheels (void);
  void grab_ball_for_travelling (void);
  void raise_arm_a_little_for_travelling (void);
  void reverse_to_line_until_middle_side_sensor_sees_it (void);
  void raise_arm_to_top_for_travelling (void);
  void move_down_line_to_next_goal_when_front_trackers_see_line (void);
  void scoot_forward_a_little_and_score_ball (void);
  void swivel_back_to_line_until_middle_side_sensor_sees_it (void);
  void rotate_right_until_front_sensors_are_on_line (void);
  void lower_arm (void);
  void drive_forward_until_ball_is_off_to_the_left (void);
  void rotate_toward_ball (void);
  void drive_forward_until_ball_is_in_clutches (void);
  void scoot_forward_a_little_more_and_collect_ball (void);
  void back_up_and_raise_arm(void);
  void scoot_forward_score_ball (void);
};

void autonomous(void);
