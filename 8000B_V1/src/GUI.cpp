#include "vex.h"
#include <cmath>

using namespace vex;


void draw_grid(void) { 
    //Brain.Screen.drawImageFromFile("meme.JPG", 0, 0);
    Brain.Screen.setPenColor(color::red);
    
    for (int i = 0; i < 2; i ++) {
        //this draws vertical lines. The X stays the same but the Y changes:
        Brain.Screen.drawLine(160*(i+1),0,160*(i+1),240);
        //this draws horizontal lines. The X changes but the Y stays the same:
        Brain.Screen.drawLine(0,80*(i+1),480,80*(i+1));
    }
}

void draw_touch() {
    Brain.Screen.setPenColor(color::red);
    //this draws a circle around the place the user is touching or last touched the LCD
    Brain.Screen.drawCircle(Brain.Screen.xPosition(),Brain.Screen.yPosition(),30);
}

void brainGUI() {
  Brain.Screen.render(true,false); //set VSync (vertical sync) on, automatic refresh to off
      //When we call Brain.Screen.render() with no arguments, it will force a redraw of the screen.
      //With the previous command here we turned off automatic redraws, so the LCD will only redraw
      //when we tell it to, which means we can guarantee it won't redraw in the middle of one of our
      //other drawing commands and give a partial picture. Getting partial pictures on redraws is
      //what causes flickering when drawing without this buffering scheme.
  while (true) {
      Brain.Screen.clearScreen(); //clears the back buffer for drawing, default clear color is black
      draw_grid(); //draws our grid to the back buffer
      Brain.Screen.render(); //flips the back buffer to the screen all at once, preventing flickering
      if (Brain.Screen.pressing()) { //if screen is touched...
          while (Brain.Screen.pressing()) { //wait until the user stops touching the screen
              Brain.Screen.clearScreen(); //while waiting, maintain the grid and draw
              draw_grid();                //a touch indicator around the user's finger
              draw_touch();
              Brain.Screen.render();
          }
          vex::task::sleep(1); //wait a second for their hand to get a little further away
          if (Brain.Screen.yPosition() > 160) {                   //released in bottom third of the screen
              if (Brain.Screen.xPosition() < 160) {                   //released in left third of the screen
                  Brain.Screen.print("bottom left");
                  //dt.turnFor(turnType::right,45,rotationUnits::deg);      //bottom left
              } else if (Brain.Screen.xPosition() > 320) {            //released in right third of the screen
                  Brain.Screen.print("bottom right");
                  //dt.turnFor(turnType::left,45,rotationUnits::deg);       //bottom right
              } else {                                                //released in middle (left-to-right) third of the screen
                  Brain.Screen.print("bottom middle");
                  //dt.driveFor(directionType::fwd,6,distanceUnits::in);    //bottom middle
              }
          } else if (Brain.Screen.yPosition() < 80) {             //released in top third of the screen
              if (Brain.Screen.xPosition() < 160) {                   //released in left third of the screen
                  Brain.Screen.print("top left");
                  //dt.turnFor(turnType::right,135,rotationUnits::deg);     //top left
              } else if (Brain.Screen.xPosition() > 320) {            //released in right third of the screen
                  Brain.Screen.print("top right");
                  //dt.turnFor(turnType::left,135,rotationUnits::deg);      //top right
              } else {                                                //released in middle (left-to-right) third of the screen
                  Brain.Screen.print("top middle");
                  //dt.driveFor(directionType::rev,6,distanceUnits::in);    //top middle
              }
          } else {                                                //released in middle (top-to-bottom) of the screen
              if (Brain.Screen.xPosition() < 160) {                   //released in left third of the screen
                  Brain.Screen.print("middle left");                
                  //dt.turnFor(turnType::right,90,rotationUnits::deg);      //middle left
              } else if (Brain.Screen.xPosition() > 320) {            //released in right third of the screen
                  Brain.Screen.print("middle right");
                  //dt.turnFor(turnType::left,90,rotationUnits::deg);       //middle right
              } else {                                                //released in middle (left-to-right) third of the screen
                  Brain.Screen.print("middle middle");
                  //do nothing                                            //middle middle
              }
          }
          //dt.stop(brakeType::coast);
      } else {
          vex::task::sleep(0.1);
      }
  }
}

