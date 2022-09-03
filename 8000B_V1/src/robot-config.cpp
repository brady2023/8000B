#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain = brain();

//Controller
competition Competition = competition();
controller  Controller1 = controller();

//Motors
/** 
BLUE = 6:1
GREEN = 18:1
RED = 36:1
**/

//chassis motors
//left
motor chassisLF = motor(vex::PORT18, vex::gearSetting::ratio6_1, true);
motor chassisLM = motor(vex::PORT19, vex::gearSetting::ratio6_1, true);
motor chassisLB = motor(vex::PORT20, vex::gearSetting::ratio6_1, false);
//right
motor chassisRF = motor (vex::PORT14, vex::gearSetting::ratio6_1, false);
motor chassisRM = motor(vex::PORT15, vex::gearSetting::ratio6_1, false);
motor chassisRB = motor (vex::PORT16, vex::gearSetting::ratio6_1, true);

//triport expander
triport expander = triport(vex::PORT11);

//pneumatics - pneumatics <name> = pneumatics(expander.<Letter>);

//sensors 
//inertial inert = inertial(vex::PORT4);

// called when program opens
void vexcodeInit(void) {
  //inert.calibrate();
  //inert.setHeading(1, degrees);
  //set motor stopping
  //lift.setStopping(hold);
  //set pistons to default stage
  //backIntake.open();
  //backClamp.open();
}