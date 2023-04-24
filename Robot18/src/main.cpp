/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    1, 2, 4, 5, 16  
// Flywheel             motor_group   8, 9            
// Indexer              digital_out   A               
// Intake_roller        motor         7               
// Expansor             motor         11              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "autonomous.h"

using namespace vex;

// A global instance of competition
competition Competition;

// Constants
double FLYWHEEL_VEL = 10000;  // mV: 10000 -> 
double EXPANSOR_DEG = -90;    // deg: -50 -> 70 -> 

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  vexcodeInit();
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  auton();
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/
bool isFlywheelRunning = false;
int Flywheeltask(){
  while(true){
    if(isFlywheelRunning){
      Flywheel.spin(fwd, FLYWHEEL_VEL, voltageUnits::mV);
    }
    task::sleep(10);
  }
}

void usercontrol(void) {
  // User control code here, inside the loop
  double turnImportance = 0.3;   // How much turning slows down the speed of forward, 0 doesn't affect, 1 stops forward
  double turnSensitivity = 0.8;  // How sensitive a turn is, 0 doesn't turn, 1 most sensitive
  Drivetrain.setStopping(coast);
  Expansor.setStopping(hold);
  
  while (1) {
    // DRIVETRAIN
    double motorTurn = Controller1.Axis1.position(percent);
    double motorMove = Controller1.Axis3.position(percent);
    // Volts range: -12 --> 12, converts percentage to volts
    double motorTurnVolts = turnSensitivity * (motorTurn * 0.12);
    // Times forward volts by a percentage from how much you turn and how important the turn is to slowing down forward speed
    double motorMoveVolts = motorMove * 0.12 * (1 - (std::abs(motorTurnVolts) / 12 * turnImportance));
    LeftDriveSmart.spin(fwd, motorMoveVolts + motorTurnVolts, volt);
    RightDriveSmart.spin(fwd, motorMoveVolts + motorTurnVolts, volt);

    // INTAKE & ROLLER  
    if (Controller1.ButtonA.pressing()){ Intake_roller.spin(fwd, 10, volt); } 
    else if (Controller1.ButtonB.pressing()){ Intake_roller.spin(fwd, 10, volt); }
    else { Intake_roller.stop(); }

    // FLYWHEEL
    if (Controller1.ButtonL2.pressing() && isFlywheelRunning == false) {
      isFlywheelRunning = true;
      wait(400, msec);
    } 
    else if (Controller1.ButtonL2.pressing() && isFlywheelRunning == true) {
      isFlywheelRunning = false;
      Flywheel.stop(brakeType::coast);
      wait(400, msec);
    }

    // EXPANSOR
    if (Controller1.ButtonX.pressing()) { Expansor.spinToPosition(EXPANSOR_DEG, rotationUnits::deg, 50, velocityUnits::pct); } 
    else { Expansor.stop(brakeType::hold); }

    // INDEXER
    if (Controller1.ButtonR2.pressing()) { Indexer.set(true); }
    else { Indexer.set(false); }

    wait(20, msec);
  }
}


int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
