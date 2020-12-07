/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Wed Sep 25 2019                                           */
/*    Description:  Creating a Start Button                                   */
/*                                                                            */
/*    This program will wait for input from the user, once the user presses   */
/*    the screen of the V5 Robot Brain, it will drive forward for six inches. */
/*                                                                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    1, 10, D
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// void BlueFront(){
//     Brain.Screen.clearScreen();
// }

// void RedFront(){
//     Brain.Screen.clearScreen();
//     Brain.Screen.setFont(fontType::mono20);
//     Brain.Screen.printAt(100, 100, "Hello");
// }

// reference:  https://www.vexforum.com/t/auton-selection-on-v5-brain/51196
// reference: https://gist.github.com/Jerrylum/cf4d417e3dbce0b7722f06b1311fae12

void Buttons(){
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.setFont(fontType::mono40);
    Brain.Screen.drawRectangle(12, 10, 105, 105, color::blue);
    Brain.Screen.drawRectangle(129, 10, 105, 105, color::blue);
    Brain.Screen.drawRectangle(246, 10, 105, 105, color::blue);
    Brain.Screen.drawRectangle(363, 10, 105, 105, color::blue);
    Brain.Screen.drawRectangle(12, 125, 105, 105, color::red);
    Brain.Screen.drawRectangle(129, 125, 105, 105, color::red);
    Brain.Screen.drawRectangle(246, 125, 105, 105, color::red);
    Brain.Screen.drawRectangle(363, 125, 105, 105, color::red);
    Brain.Screen.printAt(38, 62, "2");
    Brain.Screen.printAt(155, 62, "3");
    Brain.Screen.printAt(38, 177, "2");
    Brain.Screen.printAt(155, 177, "3");
}

// Draws a circle around the place on the screen that is touched
void HiliteTouch() {
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.drawCircle(Brain.Screen.xPosition(),Brain.Screen.yPosition(),30, color::transparent);
}

int AutonLocation(){
    
    // // Allows us to draw on the screen without the system rendering until we tell it to
    // Brain.Screen.render(true,false); 
    
    // while (1) {
    //     // Prevents flicker from constant rendering
    //     if (LocationVal == 0){
    //         Brain.Screen.clearScreen(); 
    //         Buttons();
    //         Brain.Screen.render();
    //     }
        
    //     else if (Brain.Screen.pressing() && LocationVal == 0){
    //         while(Brain.Screen.pressing()){
    //             Brain.Screen.clearScreen();
    //             Buttons();
    //             ScreenTouch();
    //             Brain.Screen.render();
    //         }
    //         task::sleep(500);

    int result = 0;
    
            if (Brain.Screen.yPosition() >= 10 && Brain.Screen.yPosition() <= 115){
                // First blue button
                if (Brain.Screen.xPosition() >= 12 && Brain.Screen.xPosition() <= 117){
                    result = 1;
                }
                // Second blue button
                else if (Brain.Screen.xPosition() >= 129 && Brain.Screen.xPosition() <= 234){
                    result = 3;
                }
            } else if (Brain.Screen.yPosition() >= 115 && Brain.Screen.yPosition() <= 230){
                // First red button
                if (Brain.Screen.xPosition() >= 12 && Brain.Screen.xPosition() <= 117){
                    result = 2;
                }
                // Second red button
                else if (Brain.Screen.xPosition() >= 129 && Brain.Screen.xPosition() <= 234){
                    result = 4;
                }
            }

    return result; // Brain.Screen.xPosition() ; //result;
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Buttons();
  while (!Brain.Screen.pressing()) task::sleep(50);
  int choice = AutonLocation();
  // while (Brain.Screen.pressing()) task::sleep(50);
  HiliteTouch();
  task::sleep(5000);
  Brain.Screen.clearScreen();
  Brain.Screen.print(choice);
  Brain.Screen.newLine();
  Brain.Screen.print(Competition.isCompetitionSwitch() || Competition.isFieldControl());
  task::sleep(50000);
}
