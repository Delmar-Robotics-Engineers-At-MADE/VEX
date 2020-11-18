#include "robot-config.h"

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VCS VEX V5                    */
/*                                                                           */
/*---------------------------------------------------------------------------*/

//Creates a competition object that allows access to Competition methods.
vex::competition    Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton( void ) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  Brain.Screen.clearLine();
  Brain.Screen.print("Running pre-auto");
    
  // find zero position for claw by checking current draw
  MotorClaw.setMaxTorque(50, vex::percentUnits::pct);
  MotorClaw.spin(vex::directionType::rev);
  while((MotorClaw.current(vex::amp)<.5)){
      vex::task::sleep(100);
  }
  MotorClaw.stop();
  MotorClaw.setRotation(0,vex::deg);
  MotorClaw.setStopping(vex::brakeType::hold);
    
  // // find zero position for shoulder by hitting bumper
  // while(! BumperA.pressing()){
  //   MotorShoulder.spin(vex::directionType::rev);
  // }
  // MotorShoulder.stop();
  // MotorShoulder.setRotation(0,vex::rotationUnits::deg);
  // MotorShoulder.setStopping(vex::brakeType::hold);
  // MotorShoulder.setVelocity(75,vex::velocityUnits::rpm);
    
  // move into threatening position briefly
  Brain.Screen.clearLine(); Brain.Screen.print("pre-auto threatening 1");
  MotorShoulder.setMaxTorque(100, vex::percentUnits::pct);
  MotorShoulder.rotateTo(150,vex::rotationUnits::deg,75,vex::velocityUnits::pct);
  Brain.Screen.clearLine(); Brain.Screen.print("pre-auto threatening 2");
  MotorClaw.rotateTo(150,vex::rotationUnits::deg);
  MotorShoulder.rotateTo(0,vex::rotationUnits::deg,50,vex::velocityUnits::pct);
  Brain.Screen.clearLine(); Brain.Screen.print("pre-auto threatening 3");
  MotorClaw.rotateTo(0, vex::rotationUnits::deg);
  MotorShoulder.stop();
  
  Brain.Screen.clearLine();
  Brain.Screen.print("Completed pre-auto");

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous( void ) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol( void ) {
  // User control code here, inside the loop

    pre_auton();
    
    Brain.Screen.clearLine();
    Brain.Screen.print("User control");

    MotorShoulder.setMaxTorque(50, vex::percentUnits::pct);

    while (1) {
        
    //Drive Control
    //Set the left and right motor to spin forward using the controller Axis values as the velocity value.
    //Since we are using a single joystick, we will need to calculate the final volicity for each motor.
    LeftMotor.spin(vex::fwd, (Controller1.Axis1.value() + Controller1.Axis2.value())/2, vex::pct); 
    RightMotor.spin(vex::fwd, (Controller1.Axis1.value() - Controller1.Axis2.value())/2, vex::pct);
    LeftMotor2.spin(vex::fwd, (Controller1.Axis1.value() + Controller1.Axis2.value())/2, vex::pct); 
    RightMotor2.spin(vex::fwd, (Controller1.Axis1.value() - Controller1.Axis2.value())/2, vex::pct);

    //Arm Control
    if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()) { // middle position
          MotorShoulder.setVelocity(50, vex::velocityUnits::pct);
          MotorShoulder.startRotateTo(300,vex::rotationUnits::deg);
    }
    else if(Controller1.ButtonR1.pressing()) { // highest position
          MotorShoulder.setVelocity(75, vex::velocityUnits::pct);
          MotorShoulder.startRotateTo(550,vex::rotationUnits::deg);
    }
    else if(Controller1.ButtonR2.pressing()) { // lowest position
          MotorShoulder.setVelocity(20, vex::velocityUnits::pct);
          MotorShoulder.startRotateTo(27,vex::rotationUnits::deg);
    }
    else { // stop
        MotorShoulder.stop();
    }

    //Claw Control
    if(Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing()) { // middle position
          MotorClaw.startRotateTo(150,vex::rotationUnits::deg);
    }
    else if(Controller1.ButtonL1.pressing()) { // closed position
          MotorClaw.startRotateTo(250,vex::rotationUnits::deg);
    }
    else if(Controller1.ButtonL2.pressing()) { // widest position
          MotorClaw.startRotateTo(0,vex::rotationUnits::deg);
    }
    else { // stop
        MotorClaw.stop();
    }

    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources.

    } // forever loop
    
    Brain.Screen.clearLine();
    Brain.Screen.print("User control complete");

}

//
// Main will set up the competition functions and callbacks.
//
int main() {
    
    //Run the pre-autonomous function. 
    pre_auton();
    
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
       
}