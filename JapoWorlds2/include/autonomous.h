#include <iostream>
#include "constants.h"
#include "robot-config.h"


using namespace vex;
extern brain Brain;

//------- Aux function definition -------//
void shoot(int volt, int discs);
void move_to_coordinate(double target_x, double target_y, double target_heading);

//--------- Main auton functions ---------//
void auton(){
  // Movement configurations
  Drivetrain.setTurnConstant(1.0);   // Recommended value 0.8 a 1.2
  Drivetrain.setTurnThreshold(1.0);  // Recommended value 2 a 5 deg

  Drivetrain.setDriveVelocity(25, percentUnits::pct);     // Set velocity to 25%
  move_to_coordinate(0, -8, 0);                           // Go to roller
  Intake_roller_group.spinFor(fwd, 100, deg, 
                              60, velocityUnits::pct);    // Move roller
  move_to_coordinate(0, 25, -10);                         // Aim to basket
  shoot(12, 2);                                           // Shoot 2 discs
  move_to_coordinate(0, -15, -130);                       // Recover position
  Drivetrain.setDriveVelocity(100, percentUnits::pct);    // Set velocity to 100%
  move_to_coordinate(0, -80, 0);                          // Go to push for 3 stack
  Drivetrain.setDriveVelocity(30, percentUnits::pct);     // Set velocity to 30%
  Intake_roller_group.spin(reverse, 10, volt);            // Start intake
  move_to_coordinate(0, 12, 0);                           // Return a little bit
  move_to_coordinate(0, -70, 110);                        // Go to collect discs and aim  100 -> 110 ->  
  Intake_roller_group.stop(hold);                         // Stop intake
  shoot(8, 3);                                            // Shoot 3 discs

  // move_to_coordinate(0, -30, 50);                         // Go for last 3 discs
  // Intake_roller_group.spin(reverse, 10, volt);            // Start intake        
  // move_to_coordinate(0, -70, 0);                          // Get last 3 discs  
}

void skills(){
}

// ---------------------------------------------------------------------
void shoot(int volts, int discs){
  Flywheel.spin(fwd, volts, voltageUnits::volt);
  wait(WAIT_UNTIL_LAUNCH*2.0, msec);
    for (int i = 0; i < discs; i++){
    Indexer.open(); 
    wait(INDEXER_WAIT*1.5, msec);
    Indexer.close();
    wait(INDEXER_WAIT*1.5, msec);
  }
  Flywheel.stop();
}

void move_to_coordinate(double target_x, double target_y, double target_heading){
  if (target_x == 0 && target_y != 0){
    Drivetrain.driveFor(fwd, target_y, distanceUnits::cm); 
  }
  if (target_y == 0 && target_x != 0){
    double ang = 90;
    if (target_x > 0){ Drivetrain.turnToHeading(ang, rotationUnits::deg); }
    if (target_x < 0){ Drivetrain.turnToHeading(-ang, rotationUnits::deg); }
    Drivetrain.driveFor(fwd, target_x, distanceUnits::cm);
  }
  if (target_x != 0 && target_y != 0){
    double ang = atan(target_y / target_x) * 180 / M_PI;
    double hyp = sqrt(target_x * target_x + target_y * target_y);
    // 1st quadrant
    if (target_x > 0 && target_y > 0){ Drivetrain.turnToHeading(ang, rotationUnits::deg); }
    // 2nd quadrant 
    if (target_x < 0 && target_y > 0){ Drivetrain.turnToHeading(-ang, rotationUnits::deg); }
    // 3rd quadrant
    if (target_x < 0 && target_y < 0){ Drivetrain.turnToHeading(180 - ang, rotationUnits::deg); }
    // 4th quadrant
    if (target_x > 0 && target_y < 0){ Drivetrain.turnToHeading(180 + ang, rotationUnits::deg); }
    Drivetrain.driveFor(hyp, distanceUnits::cm);
  }
  DrivetrainInertial.resetHeading();
  if (target_heading != 0){
    Drivetrain.turnToHeading(target_heading, rotationUnits::deg);
  }
  Drivetrain.stop(brakeType::hold);
}