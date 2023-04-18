/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       VORTEX Robotics                                           */
/*                  For more information contact A01705935@tec.mx             */
/*    Created:      15-April-2023                                             */
/*    Description:  Competition Robot 15                                      */
/*----------------------------------------------------------------------------*/


// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    1, 2, 3, 4, 5, 6, 16  
// Flywheel             motor_group   18, 19          
// Expansor             motor_group   11, 12          
// Intake_roller        motor         7               
// Indexer              digital_out   D               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include "autonomous.h"

using namespace vex;

// A global instance of competition
competition Competition; 

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

void usercontrol(void) {
  rc_auto_loop_function_Controller1();
  /*
  int Flywheel_flag = 0;
  int FLYWHEEL_VEL = 60; 
  int EXPANSOR_DEG = -90;

  double turnImportance = 0.3;   // How much turning slows down the speed of forward, 0 doesn't affect, 1 stops forward
  double turnSensitivity = 0.8;  // How sensitive a turn is, 0 doesn't turn, 1 most sensitive
  LeftFrontMotor.setStopping(brakeType::coast);
  LeftMiddleMotor.setStopping(brakeType::coast);
  LeftBackMotor.setStopping(brakeType::coast);

  RightFrontMotor.setStopping(brakeType::coast);
  RightMiddleMotor.setStopping(brakeType::coast);
  RightBackMotor.setStopping(brakeType::coast);
  Expansor.setStopping(brakeType::hold);

  while(true){
    // DRIVETRAIN
    double turnn = Controller1.Axis1.position(percent);
    double move = Controller1.Axis3.position(percent);

    double turn_volts = turnSensitivity * (turnn * 0.12);
    double move_volts = move * 0.12 * (1.0 - std::abs((turn_volts / 12.0) * turnImportance));

    LeftFrontMotor.spin(forward, move_volts + turn_volts, volt);
    LeftMiddleMotor.spin(forward, move_volts + turn_volts, volt);
    LeftBackMotor.spin(forward, move_volts + turn_volts, volt);

    RightFrontMotor.spin(forward, move_volts - turn_volts, volt);
    RightMiddleMotor.spin(forward, move_volts - turn_volts, volt);
    RightBackMotor.spin(forward, move_volts - turn_volts, volt);

    // INTAKE & ROLLER
    if (Controller1.ButtonA.pressing()){
      Intake_roller.spin(forward, 10, volt);
    } else if (Controller1.ButtonB.pressing()) {
      Intake_roller.spin(reverse, 12, volt);
    } else Intake_roller.stop();

    // FLYWHEEL
    if (Controller1.ButtonL2.pressing() && Flywheel_flag == 0) {
      Flywheel.spin(forward, FLYWHEEL_VEL, velocityUnits::pct);
      Flywheel_flag = 1;
      wait(400, msec);
    } else if (Controller1.ButtonL2.pressing() && Flywheel_flag == 1) {
      Flywheel.stop();
      Flywheel_flag = 0;
      wait(400, msec);
    }

    // INDEXER
    if (Controller1.ButtonR2.pressing()) { Indexer.set(true); }
    else { Indexer.set(false); }

    // EXPANSOR
    if (Controller1.ButtonX.pressing()) {
      Expansor.spinToPosition(EXPANSOR_DEG, rotationUnits::deg, 50, velocityUnits::pct);
    } else { Expansor.stop(brakeType::hold); }

    wait(20, msec);
  } 
  */
}


// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
