#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
vex::brain Brain;
vex::motor MotorShoulder = vex::motor(vex::PORT6,vex::gearSetting::ratio18_1);
vex::motor MotorClaw = vex::motor(vex::PORT5, vex::gearSetting::ratio18_1, false);
vex::motor LeftMotor = vex::motor(vex::PORT9, vex::gearSetting::ratio18_1);
vex::motor LeftMotor2 = vex::motor(vex::PORT10, vex::gearSetting::ratio18_1);
vex::motor RightMotor = vex::motor(vex::PORT1, vex::gearSetting::ratio18_1);
vex::motor RightMotor2 = vex::motor(vex::PORT2, vex::gearSetting::ratio18_1);
vex::bumper BumperA = vex::bumper(Brain.ThreeWirePort.A);
vex::controller Controller1 = vex::controller();
