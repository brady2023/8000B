/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       bradyc                                                    */
/*    Created:      Tue Aug 16 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

#include <string>


// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Expander19           triport       19              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "math.h"

using namespace vex;

/**TO DO:
-
**/

//-------------------------------------------------------variables and helpers---------------------------------------------------------------------------------
//math constants
const double pi = 3.1415927;
const double gPerInch = 386.04; //in inch/s/s

//Chassis Constants
const double WHEEL_RADIUS = 3.25/2.0;
const double wheelCircumference = 2 * WHEEL_RADIUS * pi;
//const double motorToLift = 72/12;
const int encoderTicksPerRev = 300;
const double ticksPerTurn = 1000;
const double turningRadius = 13.4; //tested

//Speed Constants
//const double liftSpeed = 100;
//const double ringSpeed = 100;

//Chassis States
int backState = 2; //0 is unclamped, 1 if clamp down, 2 is tilt + clamp
int chassisSpeed = 0; // 0 = regular, 1 = slow, 2 = mid

//lift states
double DEFAULT_LIFT = 0;
double HIGH_LIFT = 67.5;

//Sensitivities
const double slowSens = .25;
const double midSens = .5;
const double sens = 1; //.75 for matches, .65 for skills

//init sensitivities to regular sens
double LEFT_SENS = sens;
double RIGHT_SENS = sens;

//button booleans
bool liftMoving = false;

//absolute values
double absHeading = 0;





//-------------------------------conversion methods-------------------------------------------------------------------------------
//turns inches to rotations of the wheel
double inchToRotation(double inches){
  double temp = (inches / (wheelCircumference));
  return temp;

}

//turns degrees to radians
double degreesToRadians(double degrees){
  double temp = degrees * pi;
  return temp / 180.0;
}


//PID Methods:
//unit conversion from inches to encoder ticks
int inch2Tick (float inch) {
  int ticks;
  ticks = inch * encoderTicksPerRev /(wheelCircumference);
  return ticks;
} 

//function getting the sign of number
int signNum (double num){
  if (num < 0) return -1;
  else if (num > 0) return 1;
  else return 0;
}  

//unit conversion from degree angle to encoder ticks
//this function requires measurement of # ticks per robot turn
int degree2Tick (float degree) {//need to measure based on robot
  int ticks = degree * ticksPerTurn/encoderTicksPerRev;
  return ticks;
}

//unit conversion from degree angle to encoder ticks
//this function is based on wheel travel trajectory during a turn
//need fine tune of turningRadius to get correct angle conversion
int degree2Tick_2 (float degree) {
  float turningCirc = 2 * pi * turningRadius;
  double ticksPerTurn = inch2Tick(turningCirc);
  int ticks = degree * ticksPerTurn/360;
  return ticks; 
}

//time delay at the end of a chassis move. Minumum is 250msec regardless what the setting is
int waitTime_msec (double rawSeconds)   {
  int miliSeconds;
  miliSeconds = rawSeconds * 1000;
  if (miliSeconds < 150) //was 250
  {
   miliSeconds = 150; //was 250
  }
  return miliSeconds;
}


double Limit(double val, double min, double max){
    if(val > max) {
      return max;
    }
    else if(val < min) {
      return min;
    }
    else {
      return val;
    }
}

//-------------------------------chassis methods-------------------------------------------------------------------------------
//spins the left side of the chassis a direction and a velocity, given as pct
void leftDrive(vex::directionType type, int velocity, velocityUnits units, brakeType t) {
  chassisLF.spin(type, velocity, units);
  chassisLM.spin(type, velocity, units);
  chassisLB.spin(type, velocity, units);
}
//spins the right side of the chassis a direction and a velocity, given as pct
void rightDrive(vex::directionType type, int velocity, velocityUnits units, brakeType t) { 
  chassisRF.spin(type, velocity, units);
  chassisRM.spin(type, velocity, units);
  chassisRB.spin(type, velocity, units);
} 

void leftSpin(double velocity) {
  chassisLF.setStopping(vex::brakeType::coast);
  chassisLM.setStopping(vex::brakeType::coast);
  chassisLB.setStopping(vex::brakeType::coast);
  

  chassisLF.setVelocity(velocity,velocityUnits::rpm);
  chassisLM.setVelocity(velocity,velocityUnits::rpm);
  chassisLB.setVelocity(velocity,velocityUnits::rpm);
  
  chassisLF.spin(vex::directionType::fwd);
  chassisLM.spin(vex::directionType::fwd);
  chassisLB.spin(vex::directionType::fwd);

}

void rightSpin(double velocity) {
  chassisRF.setStopping(vex::brakeType::coast);
  chassisRM.setStopping(vex::brakeType::coast);
  chassisRB.setStopping(vex::brakeType::coast);
  
  chassisRF.setVelocity(velocity,velocityUnits::rpm);
  chassisRM.setVelocity(velocity,velocityUnits::rpm);
  chassisRB.setVelocity(velocity,velocityUnits::rpm);
  
  
  chassisRF.spin(vex::directionType::fwd);
  chassisRM.spin(vex::directionType::fwd);
  chassisRB.spin(vex::directionType::fwd);

}

//Stops the chassis using the specified brake type
void chassisStop(brakeType t){
  //stop left side
  chassisLF.stop(t);
  chassisLM.stop(t);
  chassisLB.stop(t);
  //stops right side
  chassisRF.stop(t);
  chassisRM.stop(t);
  chassisRB.stop(t);

}

void setChassisStopping(brakeType t){
    chassisLF.setStopping(t);
    chassisLM.setStopping(t);
    chassisLB.setStopping(t);
    
    chassisRF.setStopping(t);
    chassisRM.setStopping(t);
    chassisRB.setStopping(t);


}
//moves the left side of the chassis a given amount of inches at a given velocity
void leftMoveFor(double velocity, double inches){
  //converts inches to rotations of the wheel
  double rotations = inchToRotation(inches);
  //rotates left side
  chassisLF.rotateFor(fwd, rotations, rotationUnits::rev, velocity, velocityUnits::rpm, false);
  chassisLM.rotateFor(fwd, rotations, rotationUnits::rev, velocity, velocityUnits::rpm, false);
  chassisLB.rotateFor(fwd, rotations, rotationUnits::rev, velocity, velocityUnits::rpm, false);

}

//moves the right side of the chassis a given amount of inches at a given velocity
void rightMoveFor(double velocity, double inches, bool wait){
  //converts inches to rotations of the wheel
  double rotations = inchToRotation(inches);
  //rotates right side
  chassisRF.rotateFor(fwd, rotations, rotationUnits::rev, velocity, velocityUnits::rpm, false);
  chassisRM.rotateFor(fwd, rotations, rotationUnits::rev, velocity, velocityUnits::rpm, false);
  chassisRB.rotateFor(fwd, rotations, rotationUnits::rev, velocity, velocityUnits::rpm, false);
  
}


void turn(double degrees) {
  //task::sleep(15000);
  const double kP = 2.5;
  const double kD = 0;

  //set init values
  //double currHeading = inert.heading();
  //double error = (degrees - currHeading);
  //added code
  double error = degrees;
  //do left turn
  double adjust = 0;
  if (error > 180){
    adjust = 360;
  }
  error -= adjust;

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(0, 0);
  //loop --> change val to make a bit faster and less acurate
  while(fabs(error) >= .5) {

    //currHeading = inert.heading();
    //error = (degrees - currHeading);

    //if have to turn left, make error negative
    if(error < 180){
      adjust = 0;
    } else{
      adjust = 360;
    }

    

     error -= adjust;

    Brain.Screen.newLine();
    Brain.Screen.print(error);
  

    double proportion = error * kP;

    double lastError = error;
    double speed = error - lastError;

    double deriative = speed * kD;

    double power = Limit((proportion + deriative),-600,600);
    leftDrive(fwd, power, rpm, coast);
    rightDrive(reverse, power, rpm, coast);

  }
  chassisStop(brake);
}

/*void liftStop(brakeType t){
  lift.stop(t);
}

void liftSpin(double velocity){
  lift.spin(fwd, velocity, velocityUnits::pct);
}

void liftSpinFor(double deg, double velocity){
  lift.spinFor(directionType::fwd, deg, degrees, velocity, velocityUnits::pct, true);
}



void liftSpinTo(double deg, double velocity){
  int motorVal = lift.position(degrees); 
  liftSpinFor(deg * motorToLift - motorVal , velocity);
}*/

/*void switchBackClamp(){
  backState += 1;
  if(backState >= 4){
    backState = 0;
  }
  if(backState == 0){
    //fully unclamp
    backClamp.close();
  }
  else if (backState == 1) {
    //clamp but not tilt
    backClamp.open();
  }

  else if(backState == 2){
    //tilt
    backIntake.open();
  }
  else if(backState == 3){
    //untilt
    backIntake.close();
  }

}
*/





//-------------------------------------------------------buttons---------------------------------------------------------------------------------

void upPressed() {
  /*liftMoving = true;
  liftSpinTo(HIGH_LIFT, liftSpeed);
  liftMoving = false;*/
}

void downPressed() { 
  //intaking rings
  /*if(ringIntake.direction() == forward){
    ringIntake.stop(brake);
  }else{
    ringIntake.spin(directionType::fwd, ringSpeed, velocityUnits::pct);
  }*/
}

void leftPressed() {
  //outtaking rings
  /*if(ringIntake.direction() == reverse){
    ringIntake.stop(brake);
  }else{
    ringIntake.spin(directionType::rev, ringSpeed, velocityUnits::pct);
  }*/
}

void rightPressed() { 
  
  
}

void xPressed() { 
  //liftMoving = true;
  //liftSpinTo(DEFAULT_LIFT, liftSpeed);
  //liftMoving = false;

}

void yPressed() {
  

}

void aPressed() {
  
  
}

void bPressed() {
  chassisSpeed += 1;
  if(chassisSpeed > 2){
    setChassisStopping(coast);
    chassisSpeed = 0;
    LEFT_SENS = sens;
    RIGHT_SENS = sens;
  } else if (chassisSpeed == 1) {
    setChassisStopping(hold);

    LEFT_SENS = slowSens;
    RIGHT_SENS = slowSens;
  } else{
    LEFT_SENS = midSens;
    RIGHT_SENS = midSens;
  }
  
}

void l1Pressed() {


}

void l1Pressing() {
  //lift.spin(directionType::fwd, liftSpeed, velocityUnits::pct);
}

void l2Pressing() {
  //lift.spin(directionType::rev, liftSpeed, velocityUnits::pct);
}

void l2Pressed() {
  

}

void r1Pressing(){
  

}

void r2Pressing(){
  
  
}

void r1Pressed() {
  /*if(frontClamp.value()){
    frontClamp.close();
  } else {
  frontClamp.open();
  }*/
}

void r2Pressed() {
  //switchBackClamp();
}


void chassisControl() {
  //tank

  leftDrive(vex::directionType::fwd, Controller1.Axis3.value() * LEFT_SENS, velocityUnits::pct, coast);
  rightDrive(vex::directionType::fwd, Controller1.Axis2.value() * RIGHT_SENS, velocityUnits::pct, coast);
  

  //arcade
  //leftDrive(fwd, (LEFT_SENS * Controller1.Axis3.value() + RIGHT_SENS * Controller1.Axis1.value()));
  //rightDrive(fwd, (LEFT_SENS * Controller1.Axis3.value() - RIGHT_SENS * Controller1.Axis1.value()));

}


//------------------------------------PID Control Move and Turn----------------------------------------

void setChassisLSmooth(int speed){
  double inertia = .97; // was 0.5
  static int currentSpeed = 0;
  currentSpeed = inertia * currentSpeed + (1 - inertia) * speed;
  leftSpin(currentSpeed);
}

void setChassisRSmooth(int speed){
  double inertia = 0.97; // was 0.5
  static int currentSpeed = 0;
  currentSpeed = inertia * currentSpeed + (1 - inertia) * speed;
  rightSpin(currentSpeed);
}

double kP = .57; //.57 for quals
double kD = 3.4; //3.4 for quals
double C = .5; //.5 for quals

void chassisPIDMove(double inches){
   //double initHeading = inert.heading();

   double revolutions = inches / (wheelCircumference - 2.18);//wheel circumference. 
   double degrees = revolutions * 360;//How many degrees the wheels need to turn
 
   //double mtrDegrees = (degrees * 6) / 5;//How many degrees the motors need to spin
 
   chassisLF.resetRotation();
   chassisRF.resetRotation();
 
   double distanceL = chassisLF.rotation(rotationUnits::deg);
   double distanceR = chassisRF.rotation(rotationUnits::deg);
 
   double errorL = degrees - distanceL;
   double errorR = degrees - distanceL;
 
   double lastErrorL = errorL;
   double lastErrorR = errorR;
 
   double proportionalL;
   double proportionalR;
 
   double speedL;
   double speedR;
 
   double derivativeL;
   double derivativeR;
 
   double powerL;
   double powerR;
 
   while(fabs(errorL) > 20 || fabs(errorR) > 20) {
     distanceL = chassisLF.rotation(rotationUnits::deg);
     distanceR = chassisRF.rotation(rotationUnits::deg);
 
     errorL = degrees - distanceL;
     errorR = degrees - distanceL;
 
     proportionalL = errorL * kP;
     proportionalR = errorR * kP;
 
     speedL = lastErrorL - errorL;
     speedR = lastErrorR - errorR;
 
     derivativeL = -speedL * kD;
     derivativeR = -speedR * kD;
 
     lastErrorL = errorL;
     lastErrorR = errorR;

      
     powerL = Limit((proportionalL + derivativeL),-600,600);
     powerR = Limit((proportionalR + derivativeR),-600, 600);
     
     /*double currAngle = inert.heading();

     if(currAngle > initHeading + 180){
       currAngle -= 360;
     }

     double movementError = initHeading - currAngle;
     setChassisLSmooth(powerL - C * movementError);
     setChassisRSmooth(powerR + C * movementError);*/

     vex::task::sleep(10);
   }

   chassisStop(hold);
 }

/**
double kP = 20; //was .8, .5 for slow
double kD = 0; //was 1, .5 for slow
double C = 1;

void chassisPIDMove(double inches){
   double maxSpeed = 600;
   double initHeading = inert.heading();

   double acceleration = inert.acceleration(xaxis) * gPerInch;
   double velocity = 0;
   double position = 0;
 
   double error = inches - position;

   double speed;
 
   double lastError = error;

   double proportional;
 
   double derivative;
 
   double power;
    
 
   while(fabs(error) > .1) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(0, 0);
    Brain.Screen.newLine();
    Brain.Screen.print(position);


     //update vel, pos, and accel
     position += velocity * .01;
     velocity += acceleration * .01;
     acceleration = inert.acceleration(xaxis) * gPerInch;
     
     
 
     error = inches - position;
 
     proportional = error * kP;
 
     speed = lastError - error;
 
     derivative = -speed * kD;
 
     lastError = error;
      
     power = Limit((proportional + derivative), maxSpeed * -1 , maxSpeed); //* .6
 
     setChassisLSmooth(power - (C * (inert.heading() - initHeading)));
     setChassisRSmooth(power + (C * (inert.heading() - initHeading)));
     
     vex::task::sleep(10);
   }

   leftDrive(fwd, 0, velocityUnits::rpm, hold);
   rightDrive(fwd, 0, velocityUnits::rpm, hold);

   chassisStop(hold);
 } **/


/*void preAuton(void){


}

int pistons(void){
  task::sleep(500);
  //frontClamp.open(); 
  //switchBackClamp();
  //switchBackClamp();
  return 0;
}
 
//right side -- get two goals
void Auton1(void){
  inert.calibrate();
  while(inert.isCalibrating()){
    task::sleep(10);
  }

  //declare tasks
  vex::task piston = vex::task(pistons);

  //start tasks --> move to center and unclamp on the way
  piston.resume();
  chassisPIDMove(41);

  //clamp on goal
  //task::sleep(45);
  //r1Pressed();

  //move back
  chassisPIDMove(-15);

  //turn towards goal
  turn(127);

  //task::sleep(500);

  //move towards goal
  chassisPIDMove(-28.5);
  //intake goal
  //switchBackClamp();

  task::sleep(500);
  //switchBackClamp();

  chassisPIDMove(30);

    
}

//right side -- get neutral goal and our alliance goal
void Auton2(void){
  //declare tasks
  vex::task piston = vex::task(pistons);

  //start tasks --> move to center and unclamp on the way
  piston.resume();
  chassisPIDMove(41.5);
  
  
  //clamp on goal
  task::sleep(45);
  r1Pressed();

  //move back
  chassisPIDMove(-25);
  
  
}

void test(void){
  inert.calibrate();
  while(inert.isCalibrating()){
    task::sleep(10);
  }
  turn(90);
  Brain.Screen.print("HEY");
  task::sleep(2000);
  turn(0);

  //chassisPIDMove(24);
  Controller1.Screen.print("pog");
}*/


void autonomous( void ) {
  //Auton1();
  //Auton2();
  //test();
}


//-------------------------------------------------------callbacks---------------------------------------------------------------------------------


void usercontrol( void ) {

  Controller1.ButtonL1.pressed(*l1Pressed);
  Controller1.ButtonL2.pressed(*l2Pressed);
  Controller1.ButtonR1.pressed(*r1Pressed);
  Controller1.ButtonR2.pressed(*r2Pressed);
  Controller1.ButtonUp.pressed(*upPressed);
  Controller1.ButtonDown.pressed(*downPressed);
  Controller1.ButtonLeft.pressed(*leftPressed);
  Controller1.ButtonRight.pressed(*rightPressed);
  Controller1.ButtonX.pressed(*xPressed);
  Controller1.ButtonY.pressed(*yPressed);
  Controller1.ButtonA.pressed(*aPressed);
  Controller1.ButtonB.pressed(*bPressed);
  Controller1.Axis2.changed(*chassisControl);
  Controller1.Axis3.changed(*chassisControl); 

  
  //game tick
  while (1) {

    //lift stuff
    /*if ((Controller1.ButtonL2.pressing() || Controller1.ButtonL1.pressing()) && !lift.isSpinning()){
      if (Controller1.ButtonL1.pressing()){
        l1Pressing();
        } 
      else if (Controller1.ButtonL2.pressing()){
        l2Pressing();
      }
    } else if(!liftMoving){
      lift.stop();
    }*/
    /**Brain.Screen.clearScreen();
    Brain.Screen.setCursor(0, 0);
    Brain.Screen.newLine();
    Brain.Screen.print(liftEncoder.rotation(rotationUnits::deg)/ encoderToLift); **/


    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(0, 0);
    Controller1.Screen.newLine();
    std::string message = "Regular Speed";
    if(chassisSpeed == 1){
      message = "Slow Speed";
    }
    if(chassisSpeed == 2){
      message = "Mid Speed";
    }
    Controller1.Screen.print(message.c_str());

    Controller1.Screen.newLine();
    //Controller1.Screen.print(inert.heading());
    
    vex::task::sleep(20); //DO NOT DELETE -- Sleep the task for a short amount of time to prevent wasted resources. 
  }
}

//-------------------------------------------------------main-------------------------------------------------------------------------------

int main() {
    vexcodeInit();
    brainGUI();
    Competition.drivercontrol(usercontrol);
    Competition.autonomous(autonomous);
    
}
