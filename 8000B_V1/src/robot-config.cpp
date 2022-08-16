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
motor chassisLF = motor(vex::PORT9, vex::gearSetting::ratio6_1, false);
motor chassisLM = motor(vex::PORT3, vex::gearSetting::ratio6_1, false);
motor chassisLB = motor(vex::PORT7, vex::gearSetting::ratio6_1, false);
//right
motor chassisRF = motor (vex::PORT10, vex::gearSetting::ratio6_1, true);
motor chassisRM = motor(vex::PORT1, vex::gearSetting::ratio6_1, true);
motor chassisRB = motor (vex::PORT2, vex::gearSetting::ratio6_1, true);

//triport expander
triport expander = triport(vex::PORT11);

//pneumatics - pneumatics <name> = pneumatics(expander.<Letter>);

//sensors 
inertial inert = inertial(vex::PORT4);

// called when program opens
void vexcodeInit(void) {
  //inert.calibrate();
  inert.setHeading(1, degrees);
  //set motor stopping
  //lift.setStopping(hold);
  //set pistons to default stage
  //backIntake.open();
  //backClamp.open();
}