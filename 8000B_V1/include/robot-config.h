using namespace vex;

//Brain
extern brain Brain;

//Controller
extern competition Competition;
extern controller  Controller1;

//Motors

//chassis motors
//left
extern motor chassisLF;
extern motor chassisLM;
extern motor chassisLB;
//right
extern motor chassisRF;
extern motor chassisRM;
extern motor chassisRB;

//motor - extern motor <name>
//pneumatics - extern pneumatics <name>

//triport expander
extern triport expander;

//sensors
//extern inertial inert;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
