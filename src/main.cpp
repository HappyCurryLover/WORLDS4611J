/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       rishs                                                     */
/*    Created:      9/14/2024, 1:59:02 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include <iostream>
using namespace vex;

// A global instance of competition
competition Competition;

// this section defines global variables (variables that are used across different functions) and also the motors and vex parts
vex::brain     Brain;
motor leftF = motor(PORT7, true);
motor leftT = motor(PORT8, false);
motor leftB = motor(PORT9, true);

motor rightF = motor(PORT11, false);
motor rightT = motor(PORT12, true);
motor rightB = motor(PORT13, false);

motor lbL = motor(PORT2, true);
motor lbR = motor(PORT19, false);
motor intakeMotor = motor(PORT5, false);

inertial inert = inertial(PORT13);
optical optic = optical(PORT10);
distance conveyorLidar = distance(PORT3);
distance mogoLidar = distance(PORT6);

controller c = controller();
digital_out clamp = digital_out(Brain.ThreeWirePort.B);
digital_out intakeLift = digital_out(Brain.ThreeWirePort.C);
digital_out doinkL = digital_out(Brain.ThreeWirePort.D);
digital_out doinkR = digital_out(Brain.ThreeWirePort.G);

double gAngle = inert.rotation(degrees);
const int tile = 345;
int current_auton_selection = 0;
bool auto_started = false;
bool blueSort = false;
bool redSort = false;



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

/* this does things before the actual match; basically you have to get to the actual match AT LEAST 3 seconds
   before the actual match starts, because you need to turn on and run the bot so that it has 3 seconds to calibrate
   an easier way to explain it is that this code runs before the referees turn on the auton part of the comp switch */
void pre_auton(void) {
  inert.calibrate(1);
  wait(3,sec);

  while(!auto_started){
    Brain.Screen.setFont(vex::fontType::mono60);
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(270, 10, 80, 80);
    Brain.Screen.drawRectangle(270, 95, 80, 80);
    Brain.Screen.printAt(270+25, 10+55, "-");
    Brain.Screen.printAt(270+25, 95+55, "+");
    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(355, 10, 80, 80);
    Brain.Screen.drawRectangle(355, 95, 80, 80);
    Brain.Screen.printAt(355+25, 10+55, "-");
    Brain.Screen.printAt(355+25, 95+55, "+");

    Brain.Screen.setFillColor(black);
    Brain.Screen.setFont(vex::fontType::mono30);
    Brain.Screen.printAt(5, 170, "SELECTED AUTON:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 210, "1. RED POSITIVE ");
        break;
      case 1:
        Brain.Screen.printAt(5, 210, "2. RED NEGATIVE ");
        break;
      case 2:
        Brain.Screen.printAt(5, 210, "3. BLUE POSITIVE");
        break;
      case 3:
        Brain.Screen.printAt(5, 210, "4. BLUE NEGATIVE");
        break;
      case 4:
        Brain.Screen.printAt(5, 210, "5. AUTON SKILLS ");
        break;
      case 5:
        Brain.Screen.printAt(5, 210, "6. PRELOAD RPOS ");
        break;
      case 6:
        Brain.Screen.printAt(5, 210, "7. PRELOAD RNEG ");
        break;
      case 7:
        Brain.Screen.printAt(5, 210, "8. PRELOAD BPOS ");
        break;
      case 8:
        Brain.Screen.printAt(5, 210, "9. PRELOAD BNEG ");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;

      // if (270 < Brain.Screen.xPosition() < 350 && 95 < Brain.Screen.yPosition() < 175) {
      //   current_auton_selection = 0;
      // }
      // if (270 < Brain.Screen.xPosition() < 350 && 10 < Brain.Screen.yPosition() < 90) {
      //   current_auton_selection = 1;
      // }
      // if (355 < Brain.Screen.xPosition() < 435 && 95 < Brain.Screen.yPosition() < 175) {
      //   current_auton_selection = 2;
      // }
      // if (355 < Brain.Screen.xPosition() < 435 && 10 < Brain.Screen.yPosition() < 90) {
      //   current_auton_selection = 3;
      // }

    }
    else if (current_auton_selection == 9){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}




/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/


/* the next few functions are the basic auton components as shown by their names
   they are all coded in PID except for regLeft(), which is a test i was trying for auton
   the code was getting stuck in the PID turnLeft function, as i wrote below, so I was 
   going to try using regular left turning but I didn't get a chance to. You can use the 2 hours 
   either trying to get turning to work with auton or practicing driving strategy */

void drive(double target, double max = 40) {
  leftF.resetPosition();
  rightF.resetPosition();
  target *= tile;
  double derivative;
  double error = target - (leftF.position(degrees) + rightF.position(degrees))/2;
  double kp = 1.2;
  double kd = 0.5;

  while (fabs(error) > 3) {
    double previousError = error;
    error = target - (leftF.position(degrees) + rightF.position(degrees))/2;
    derivative = error - previousError;

    double speed = error*kp - derivative*kd;
    if (speed > max) speed = max;
    if (speed < -max) speed = -max;

    leftT.spin(forward, speed, pct);
    leftF.spin(forward, speed, pct);
    leftB.spin(forward, speed, pct);
    rightF.spin(forward, speed, pct);
    rightT.spin(forward, speed, pct);
    rightB.spin(forward, speed, pct);
    wait(10, msec);
  }

  leftF.stop(brake);
  leftT.stop(brake);
  leftB.stop(brake);
  rightF.stop(brake);
  rightT.stop(brake);
  rightB.stop(brake);
}

void driveReg(double target, double max = 40) {
  leftF.resetPosition();
  rightF.resetPosition();
  target *= tile;
  double error = target - (leftF.position(degrees) + rightF.position(degrees))/2;

  while (fabs(error) > 3) {
    error = target - (leftF.position(degrees) + rightF.position(degrees))/2;

    double speed = max;
    if (speed > max) speed = max;
    if (speed < -max) speed = -max;

    leftT.spin(forward, speed, pct);
    leftF.spin(forward, speed, pct);
    leftB.spin(forward, speed, pct);
    rightF.spin(forward, speed, pct);
    rightT.spin(forward, speed, pct);
    rightB.spin(forward, speed, pct);
    wait(10, msec);
  }

  leftF.stop(brake);
  leftT.stop(brake);
  leftB.stop(brake);
  rightF.stop(brake);
  rightT.stop(brake);
  rightB.stop(brake);
}

// this is a very intelligent piece of code that makes the robot turn left and makes the team very happy because it allows our robot to turn left to grab the points and make us very hpapy
void turnLeft(double angle, int max = 100) {
  inert.resetRotation();
  double kp = 0.3;
  // double ki = 0.1;
  double error = angle + inert.rotation(degrees);
  double kd  = 0.2;
  double derivative = 0.0;
  // double integral = 0.0;
  
  while (fabs(error) > 2) {
    double previousError = error;
    error = angle + inert.rotation(degrees);
    derivative = error - previousError;
    // integral += error;

    double speed = error*kp - derivative*kd;
    if (speed > max) {speed = max;}

    leftF.spin(reverse, speed, pct);
    leftB.spin(reverse, speed, pct);
    leftT.spin(reverse, speed, pct);
    rightF.spin(forward, speed, pct);
    rightT.spin(forward, speed, pct);
    rightB.spin(forward, speed, pct);
  }

    leftF.stop(brake);
    leftT.stop(brake);
    leftB.stop(brake);
    rightF.stop(brake);
    rightT.stop(brake);
    rightB.stop(brake);
}

void turnRight(double angle, int max = 100) {
  inert.resetRotation();
  double kp = 0.3;
  // double ki = 0.1;
  double error = angle - inert.rotation(degrees);
  double kd  = 0.25;
  double derivative = 0.0;
  // double integral = 0.0;
  
  while (fabs(error) > 2) {
    double previousError = error;
    error = angle - inert.rotation(degrees);
    derivative = error - previousError;
    // integral+=error;

    double speed = error*kp - derivative*kd;
    if (speed > max) {speed = max;}

    leftF.spin(forward, speed, pct);
    leftB.spin(forward, speed, pct);
    leftT.spin(forward, speed, pct);
    rightF.spin(reverse, speed, pct);
    rightT.spin(reverse, speed, pct);
    rightB.spin(reverse, speed, pct);
  }

    leftF.stop(brake);
    leftT.stop(brake);
    leftB.stop(brake);
    rightF.stop(brake);
    rightT.stop(brake);
    rightB.stop(brake);
}

// void turnRightReg(double angle) {
//   inert.resetRotation();
//   double error = angle - inert.rotation(degrees);
//   while (fabs(error) > 0) {
//     error = angle - inert.rotation(degrees);
//     // integral+=error;
//     double speed = 30;
//     leftF.spin(forward, speed*0.5, pct);
//     leftB.spin(forward, speed*0.5, pct);
//     leftT.spin(forward, speed*0.5, pct);
//     rightF.spin(reverse, speed*0.5, pct);
//     rightT.spin(reverse, speed*0.5, pct);
//     rightB.spin(reverse, speed*0.5, pct);
//   }
//     leftF.stop(brake);
//     leftT.stop(brake);
//     leftB.stop(brake);
//     rightF.stop(brake);
//     rightT.stop(brake);
//     rightB.stop(brake);
// }

void intake() {
  intakeMotor.resetPosition();
  intakeMotor.spin(forward, 100, pct);
  double pos = intakeMotor.position(degrees);
  while (true) {
    double prevPos = pos;
    wait(1, sec);
    pos = intakeMotor.position(degrees);
    if (fabs(prevPos - pos) < 3) {
      intakeMotor.spin(reverse, 100, pct);
      wait(0.3, sec);
      intakeMotor.spin(forward, 100, pct);
    }
  }
}


void brainDisplay() {
  while (true) {
    Brain.Screen.setFont(vex::fontType::mono15);
    Brain.Screen.printAt(10, 30, "Global Left %f", leftB.position(degrees));
    Brain.Screen.printAt(10, 40, "GLobal Right %f", rightB.position(degrees));
    Brain.Screen.printAt(10, 50, "Left %f", leftF.position(degrees));
    Brain.Screen.printAt(10, 60, "Right %f", rightF.position(degrees));
    Brain.Screen.printAt(10, 70, "Inertial %f", inert.rotation(degrees));
    Brain.Screen.printAt(10, 80, "LB %f", lbL.position(degrees));
    Brain.Screen.printAt(10, 90, "Conveor %f", intakeMotor.position(degrees));
    wait(20, msec);
  }
}



// ============================================== AUTON FUNCTIONS ===================================================

/* the next 4 functions are for the auton for each respective corner. 
   the issue that wasn't working was that the code kept getting stuck in the turning functions
   the best way i think you can solve this is by using normal turn instead of PID turning, and 
   i made a new turning function called regLeft() instead of turnLeft(), so try using that 
   instead */

// void redPos() { //corner rush idea
//   driveReg(1.6, 50);
//   doink.set(true);
//   wait(75, msec);
//   drive(-0.33);
//   wait(75, msec);
//   doink.set(false);
//   wait(150, msec);
//   turnRight(89);
//   drive(-1.2, 20);
//   clamp.set(true);
//   intakeMotor.spin(forward, 100, pct);
//   wait(1, sec);
//   turnRight(180);
//   drive(0.5);
// }

// // PATH DOES NOT WORK !
// void diagonalGoalrushRed() {
//   driveReg(1.7, 60);
//   doink.set(true);
//   drive(-1);
//   doink.set(false);
//   turnLeft(180);
//   drive(-0.5);
//   clamp.set(true);
//   // intakeMotor.spin(forward, 100, pct);
//   // clamp.set(false);
//   // turnLeft(100);
//   // drive(0.5);
//   // drive(-1.3);
//   // clamp.set(true);
//   // turnLeft(90);
//   // drive(0.8);
// }

// void blueNeg() {
//   moveSixbar(forward);
//   drive(0.3, 20);
//   moveSixbar(reverse);
//   drive(-1.5, 25);
//   // turnLeft(30);
//   clamp.set(true);
//   turnLeft(120);
//   intakeMotor.spin(forward, 100, pct);
//   drive(0.7, 30);
//   turnLeft(90);
//   drive(0.5, 20);
//   wait(2, sec);
//   turnLeft(91, 80);
//   drive(0.5, 30);
// }

// void bluePos() {
//   driveReg(1.6, 50);
//   doink.set(true);
//   wait(75, msec);
//   drive(-0.33);
//   wait(75, msec);
//   doink.set(false);
//   wait(150, msec);
//   turnLeft(89);
//   drive(-1.2, 20);
//   clamp.set(true);
//   intakeMotor.spin(forward, 100, pct);
//   wait(1, sec);
//   turnLeft(180);
//   drive(0.5);
// }

// void redNeg() {
//   moveSixbar(forward);
//   drive(0.25, 20);
//   moveSixbar(reverse);
//   drive(-1.15, 25);
//   turnRight(30);
//   drive(-0.45);
//   clamp.set(true);
//   turnRight(90);
//   intakeMotor.spin(forward, 100, pct);
//   drive(0.7, 30);
//   turnRight(90);
//   drive(0.5, 20);
//   wait(2, sec);
//   turnRight(91, 80);
//   drive(0.5, 30);
// }


// void test() { // auton testing
//   moveSixbar(forward);
//   wait(0.2, sec);
//   moveSixbar(reverse);
//   wait(0.2, sec);
//   moveSixbar(forward);
// }


// void autonSkills() { // unfinished skills auton (git says consistent 11 inconsistent 22 but i dont remembr the actual run) 
//   drive(-0.45, 20);
//   clamp.set(true);
//   intakeMotor.spin(forward, 100, pct);
//   wait(0.2, sec);
//   turnRight(90);
//   drive(1, 30);
//   wait(0.5, sec);
//   wait(0.3, sec);
//   drive(0.3, 25);
//   wait(1, sec);
//   turnRight(110);
//   clamp.set(false);
//   wait(.1, sec);
//   drive(0.075);
//   turnLeft(110);
//   drive(-3.45, 25);
//   turnLeft(109);
//   drive(-3.43, 25);
//   clamp.set(true);
//   intakeMotor.spin(forward, 100, pct);
//   turnRight(90); 
//   drive(1);
//   turnRight(90);
//   drive(1.5);
//   turnRight(135);
//   turnRight(120);
//   drive(-0.5);
//   clamp.set(false);
// }

// void idealAuton() {
//   intakeMotor.spin(forward, 100, pct);
//   turnRight(120);
//   drive(-1.2);
//   clamp.set(true);
//   turnRight(150);
//   drive(1.5);
//   turnRight(120);
//   drive(-1);
//   clamp.set(false);
//   drive(1);
//   turnLeft(120);
//   drive(-3.5);
//   clamp.set(true);
//   turnRight(180);
//   drive(1.5);
//   turnLeft(120);
//   drive(-1);
//   clamp.set(false);
//   turnRight(3);
//   drive(4.5);
//   turnRight(180);
//   drive(-1.5);
//   clamp.set(true);
//   turnRight(100);
//   drive(-1.5);
//   clamp.set(false);
//   drive(1.5);
//   turnRight(180);
//   drive(-2);
//   turnRight(10);
//   drive(-2);
// }

// void autonSkills() { // unfinished skills auton
//   drive(-0.45, 20);
//   clamp.set(true);
//   intakeMotor.spin(forward, 100, pct);
//   wait(0.2, sec);
//   turnRight(90);
//   drive(1, 30);
//   wait(0.3, sec);
//   drive(0.3, 25);
//   wait(1, sec);
//   turnRight(110);
//   drive(1.4, 30);
//   turnRight(10);
//   wait(0.3, sec);
//   drive(-1.9);
//   intakeMotor.stop(brake);
//   wait(.1, sec);
//   clamp.set(false);
//   wait(.1, sec);
//   drive(0.075);
//   turnLeft(107);
//   drive(-3.45, 25);
//   clamp.set(true);
//   intakeMotor.spin(forward, 100, pct);
//   turnRight(90);
//   drive(1);
//   turnRight(90);
//   drive(1);
//   turnRight(90);
//   drive(1.5);
//   turnRight(120);
//   drive(-0.5);
//   clamp.set(false);
// }

// void autonSkills2() {
//   intakeMotor.spin(forward, 70, pct);
//   turnLeft(116.5);
//   drive(-1, 20);
//   clamp.set(true);
//   turnRight(16.5);
//   drive(0.2);
//   turnRight(180-26.5);
//   drive(3.7);
//   turnRight(180-26.5);
//   drive(3.5);
//   turnRight(180-26.5);
//   clamp.set(false);
//   turnRight(90-16.5);
//   drive(6);
//   wait(0.2, sec);
//   turnLeft(10);
//   drive(1.3);
//   doink.set(true);
//   turnLeft(170);
//   drive(1);
//   doink.set(false);
// //20
//   drive(-3.5, 25);
//   clamp.set(true);
//   turnRight(153);
//   drive(3.2);
//   wait(0.3, sec);
//   drive(-0.5);
//   turnLeft(30);
//   drive(3);
//   turnRight(200);
//   drive(-0.3);
//   clamp.set(false);
// }



// void preloadREDPOS() {
//   drive(-1.4, 25); 
//   clamp.set(true);
//   intakeMotor.spin(forward, 70, pct);
//   turnLeft(90);
//   drive(1);
//   wait(0.5, sec);
//   turnRight(175);
//   driveReg(1.65, 30);
//   wait(1.5, sec);
//   intakeMotor.spin(reverse, 100, pct);
//   wait(0.2, sec);
//   intakeMotor.stop(brake);
// }
// void preloadREDNEG() {
//   drive(-1.4, 25);
//   clamp.set(true);
//   intakeMotor.spin(forward, 70, pct);
//   wait(0.1, sec);
//   // drive(0.2, 70); // TEST THIS PART SATURDAY
//   // drive(-0.2, 70); // TEST THSI PART SATURDAY most likely dont need bc clamp fixed but idk
//   wait(0.3, sec);
//   turnRight(90);
//   drive(1);
//   wait(1.8, sec);
//   turnRight(90);
//   drive(0.6, 15);
//   wait(0.3, sec);
//   drive(-0.6);
//   turnRight(85);
//   driveReg(1.65, 30);
//   wait(1.5, sec);
//   intakeMotor.spin(reverse, 100, pct);
//   wait(0.2, sec);
//   intakeMotor.stop(brake);
// }
// void preloadBLUEPOS() {
//   drive(-1.4, 25); 
//   clamp.set(true);
//   intakeMotor.spin(forward, 70, pct);
//   turnRight(90);
//   drive(1);
//   wait(0.5, sec);
//   turnLeft(175);
//   driveReg(1.65, 30);
//   wait(1.5, sec);
//   intakeMotor.spin(reverse, 100, pct);
//   wait(0.2, sec);
//   intakeMotor.stop(brake);
// }
// void preloadBLUENEG() {
//   drive(-1.4, 25);
//   clamp.set(true);
//   intakeMotor.spin(forward, 70, pct);
//   wait(0.1, sec);
//   // drive(0.2, 70); // TEST THIS PART SATURDAY
//   // drive(-0.2, 70); // TEST THSI PART SATURDAY most likely dont need bc clamp fixed but idk
//   wait(0.3, sec);
//   turnLeft(90);
//   drive(1);
//   wait(1.8, sec);
//   turnLeft(90);
//   drive(0.6, 15);
//   wait(0.3, sec);
//   drive(-0.6);
//   turnLeft(85);
//   driveReg(1.6, 30);
//   wait(1.5, sec);
//   intakeMotor.spin(reverse, 100, pct);
//   wait(0.2, sec);
//   intakeMotor.stop(brake);
// }



// /*---------------------------------------------------------------------------*/
// /*                                                                           */
// /*                               TASK FUNCTIONS                              */
// /*                                                                           */
// /*---------------------------------------------------------------------------*/


void lbSet() {
  double pos = lbL.position(degrees);
  while (fabs(pos) < 40) {
    lbL.spin(forward, 80, pct);
    lbR.spin(forward, 80, pct);
  }
  lbL.stop(hold);
  lbR.stop(hold);
}

void lbScore() {
  double pos = lbL.position(degrees);
  while (fabs(pos) < 570) {
    lbL.spin(forward, 100, pct);
    lbR.spin(forward, 100, pct);
  }
  lbL.stop(hold);
  lbR.stop(hold);
}

void lbReset() {
  double pos = lbL.position(degrees);
  while (fabs(pos) > 0) {
    lbL.spin(reverse, 100, pct);
    lbR.spin(reverse, 100, pct);
  }
  lbL.stop(hold);
  lbR.stop(hold);
}

void colorSortRing() {
  wait(18, msec);
  intakeMotor.spin(reverse, 100, pct);
  wait(5, msec);
  intakeMotor.spin(forward, 100, pct);
}



void ladyBrownDrive() {
  bool lbBool = false;
  while (true) {
    if (c.ButtonY.PRESSED) {thread s(lbSet);}
    if (c.ButtonB.PRESSED) {
      if (lbBool) {thread c(lbScore);}
      else {
        thread r(lbReset);}
    }
  }
}

void colorSort() {
  bool colorsort = false;
  bool blue = false;
  bool red = false;
  while (true) {
    if (c.ButtonA.PRESSED) {colorsort = true;}


    if (optic.hue() > 180) {blue = true;}
    else {blue = false;}
    if (optic.hue() < 25) {
      red = true;
    }
    else {red = false;}

    if (colorsort) {
      if (conveyorLidar.objectDistance(mm) < 48) {
        if ((blueSort && blue) || (redSort && red)) {
          thread c(colorSortRing);
          colorsort = false;
        }
      }
    }
  }
}

// void autoClamp() {
//   if (mogoLidar.) {

//   }
// }


// // driving control function: call me to explain the code if you need to adjust drive speed or anything else
void usercontrol(void) {
  // User control code here, inside the loop
  int intakeSpeed = 100;
  bool intakeBool = false;
  bool clampBool = true;
  bool doinkBool = true;

  while (1) {
    int logFD = (c.Axis3.position()*c.Axis3.position()) / 150;
    if (c.Axis3.position() < 0) {
      logFD *= -1;
    }
    int logLR = c.Axis1.position()*c.Axis1.position() / 120;
    if (c.Axis1.position() < 0) {
      logLR *= -1;
    }

    thread b(ladyBrownDrive);
    thread q(colorSort);

    if (c.ButtonL1.PRESSED) {intakeMotor.spin(forward, intakeSpeed, pct);}
    if (c.ButtonL2.PRESSED) {intakeMotor.spin(reverse, 100, pct);}
    if (c.ButtonL1.RELEASED) {intakeMotor.stop();}
    if (c.ButtonL2.RELEASED) {intakeMotor.stop();}


    if (c.ButtonR1.PRESSED) {
      clamp.set(!clampBool);
      clampBool = !clampBool;
    }

    if (c.ButtonX.PRESSED) {
      doinkR.set(!doinkBool);
      doinkBool = !doinkBool;
    }
    

    leftF.spin(forward, (logFD + logLR), pct);
    leftT.spin(forward, (logFD + logLR), pct);
    leftB.spin(forward, (logFD + logLR), pct);
    rightF.spin(forward, (logFD - logLR), pct);
    rightT.spin(forward, (logFD - logLR), pct);
    rightB.spin(forward, (logFD - logLR), pct);

    wait(20, msec);
  }
}


/* this function is the thing that lets you have multiple slots for auton (clicking the brain switches slots and switches auton)
   if you get auton to work, this is where you should put in the code for the different corners and which slot you want each auton to go in.
   it is currently coded to have the defaultAuton(), driving forward 5 inches, in the first slot, so you will not have to click the brain to 
   select the first auton */



// NEEV AND AADITYA (or whoever is in drivebox): when it says something like "case 4", that means the Brain will read AUTON 5. 
// this is because code counts starting from 0, not 1, but whether or not you understand it, keep this in mind if I am not at a competition.
void autonomous(void) {
  auto_started = true;
  switch(current_auton_selection){ 
    case 0:
      // redPos();
      break;
    case 1:
      // redNeg();/
      break;
    case 2:
      // bluePos();
      break;
    case 3:
      // blueNeg();
      break;
    case 4:
      // idealAuton();
      break;
    case 5:
      // preloadREDPOS();
      break;
    case 6:
      // preloadREDNEG();
      break;
    case 7:
      // preloadBLUEPOS();
      break;
    case 8:
      // preloadBLUENEG();
      break;
  }
} 

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  thread t(brainDisplay);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}