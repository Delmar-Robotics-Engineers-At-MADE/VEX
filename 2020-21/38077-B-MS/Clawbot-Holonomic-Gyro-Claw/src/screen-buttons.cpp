#include "vex.h"
#include "screen-buttons.h"

// which autonomous variation to run
int AutonChoice = AUTON_DO_NOTHING;

// reference:  https://www.vexforum.com/t/auton-selection-on-v5-brain/51196
// reference: https://gist.github.com/Jerrylum/cf4d417e3dbce0b7722f06b1311fae12

void DrawButtons(){
    Brain.Screen.clearScreen();
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.setFont(fontType::mono30);
    Brain.Screen.drawRectangle(12, 10, 105, 105, color::blue);
    Brain.Screen.drawRectangle(129, 10, 105, 105, color::blue);
    Brain.Screen.drawRectangle(246, 10, 105, 105, color::blue);
    Brain.Screen.drawRectangle(363, 10, 105, 105, color::blue);
    Brain.Screen.drawRectangle(12, 125, 105, 105, color::red);
    Brain.Screen.drawRectangle(129, 125, 105, 105, color::red);
    Brain.Screen.drawRectangle(246, 125, 105, 105, color::red);
    Brain.Screen.drawRectangle(363, 125, 105, 105, color::red);
    Brain.Screen.printAt(38, 62, "L 1");
    Brain.Screen.printAt(155, 62, "L 2"); // + 117
    Brain.Screen.printAt(272, 62, "R 1");
    Brain.Screen.printAt(389, 62, "R 2");
    Brain.Screen.printAt(38, 177, "L 1");
    Brain.Screen.printAt(155, 177, "L 2");
    Brain.Screen.printAt(272, 177, "R 1");
    Brain.Screen.printAt(389, 177, "R 2");
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

    int result = AUTON_DO_NOTHING;
    
            if (Brain.Screen.yPosition() >= 10 && Brain.Screen.yPosition() <= 115){
                // First blue button
                if (Brain.Screen.xPosition() >= 12 && Brain.Screen.xPosition() <= 117){
                    result = AUTON_BLUE_LEFT_1_BALL;
                }
                // Second blue button
                else if (Brain.Screen.xPosition() >= 129 && Brain.Screen.xPosition() <= 234){ // + 12, + 105
                    result = AUTON_BLUE_LEFT_2_BALLS;
                }
                // third blue button
                else if (Brain.Screen.xPosition() >= 246 && Brain.Screen.xPosition() <= 351){
                    result = AUTON_BLUE_RIGHT_1_BALL;
                }
                // fourth blue button
                else if (Brain.Screen.xPosition() >= 363 && Brain.Screen.xPosition() <= 468){
                    result = AUTON_BLUE_RIGHT_2_BALLS;
                }
            } else if (Brain.Screen.yPosition() >= 115 && Brain.Screen.yPosition() <= 230){
                // First red button
                if (Brain.Screen.xPosition() >= 12 && Brain.Screen.xPosition() <= 117){
                    result = AUTON_RED_LEFT_1_BALL;
                }
                // Second red button
                else if (Brain.Screen.xPosition() >= 129 && Brain.Screen.xPosition() <= 234){
                    result = AUTON_RED_LEFT_2_BALLS;
                }
                // third red button
                else if (Brain.Screen.xPosition() >= 246 && Brain.Screen.xPosition() <= 351){
                    result = AUTON_RED_RIGHT_1_BALL;
                }
                // fourth red button
                else if (Brain.Screen.xPosition() >= 363 && Brain.Screen.xPosition() <= 468){
                    result = AUTON_RED_RIGHT_2_BALLS;
                }
            }

    return result; // Brain.Screen.xPosition() ; //result;
}
