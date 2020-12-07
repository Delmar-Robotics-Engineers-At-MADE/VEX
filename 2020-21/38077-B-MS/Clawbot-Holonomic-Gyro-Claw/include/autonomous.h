#define LINESPEED 30
#define LINECORRECT 10
#define LINETHRESHOLD 8

#define LINELOST 0
#define LINECENTERED 1
#define LINEOFFTOLEFT 2
#define LINEOFFTORIGHT 3
#define LINETHICKOFFTOLEFT 4
#define LINETHICKOFFTORIGHT 5
#define LINEALLON 6

// flag so pre_auton runs before tele-op when off competition switch
extern bool pre_auton_done;

void autonomous(void);