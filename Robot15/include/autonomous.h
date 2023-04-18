/*----------------------------------------------------------------------------*/
/*    Module:       autonomous.h                                              */
/*    Author:       VORTEX Robotics                                           */
/*                  For more information contact A01705935@tec.mx             */
/*    Created:      18-feb-2023                                               */
/*    Description:  Default header for V5 projects                            */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "robot-config.h"
using namespace vex;

// Set up devices and constants
extern brain Brain;

/************ Constants ****************/
const double INDEXER_GO        = 1200;     //ms
const double INDEXER_BACK      = 1000;     //ms            
const double WAIT_UNTIL_LAUNCH = 2000;     //ms  3000 -> 5000 -> 

//-------------------------------------------------------------------------
void shoot_disc(double vel, int t,  int n, int t1, int t2){
  /* Shoot the disc collected with a given velocity (vel) in percentage
   * and wait a t time to activate the indexer and make n throws and 
   * keep it open a t1 time and t2 close. */
  for (int i = 0; i<n; i++){
    Flywheel.spin(forward, vel, velocityUnits::pct);
    wait(t, msec);
    Indexer.open();
    wait(t1, msec);
    Indexer.close();
    wait(t2, msec);
    t = 0;
    vel = vel - 6;
  }
  Flywheel.stop(brakeType::coast);
}

void reset_turnH(int d, int vel){
  DrivetrainInertial.calibrate();
  Drivetrain.turnToHeading(d, deg, vel, velocityUnits::pct);
}

void activate_intake(double rot, double vel){
  /* Activate intake and roller motors to move certain degrees with specific velocity */
  Intake_roller.spinFor(forward, rot, rotationUnits::deg, vel, velocityUnits::pct);
}

void auton(){
  /* Step 1 */
  Drivetrain.driveFor(directionType::rev, 3, distanceUnits::cm, 15, velocityUnits::pct);  // Go to the roller
  activate_intake(360, 90);  // Move the roller 
  /* Step 2 */
  Drivetrain.driveFor(directionType::fwd, 10, distanceUnits::cm, 20, velocityUnits::pct);  // Go to throw discs
  Drivetrain.turnToHeading(-4, rotationUnits::deg, 3, velocityUnits::pct);  // Turn to the basket
  shoot_disc(74, WAIT_UNTIL_LAUNCH, 2, INDEXER_BACK, INDEXER_GO);  // Shoot the discs 
  /* Step 3
  Drivetrain.driveFor(directionType::rev, 4.5, distanceUnits::cm, 10, velocityUnits::pct); // Go for the rest of discs
  reset_turnH(-104, 30); 
  Drivetrain.driveFor(directionType::rev, 62, distanceUnits::cm, 80, velocityUnits::pct);
  Intake_roller.spin(forward, 95, velocityUnits::pct);  // Activate intake to collect the discs
  Drivetrain.driveFor(directionType::rev, 50, distanceUnits::cm, 25, velocityUnits::pct);  // Collecting the discs
  wait(5000, msec);      // wait for 5 seconds
  Intake_roller.stop();  // Stop the intake
  reset_turnH(80, 30);   // Aim for the basket
  Drivetrain.driveFor(directionType::fwd, 15, distanceUnits::cm, 20, velocityUnits::pct);  // Move closer to the basket
  shoot_disc(75, WAIT_UNTIL_LAUNCH, 3, INDEXER_BACK, INDEXER_GO);  // Shoot the discs  */
}


void skills(){
  auton();
  /*Step 4 */
  reset_turnH(-95, 30);  // Go for the second roller
  Drivetrain.driveFor(directionType::fwd, 50, distanceUnits::cm, 40, velocityUnits::pct); 
  reset_turnH(-110, 30); 
  Drivetrain.driveFor(directionType::rev, 150, distanceUnits::cm, 50, velocityUnits::pct); 
  activate_intake(90, 90);
}

/* Record of the values
 - Step 1
  + 5 cm, 30 pct     -> 3 cm, 15 pct     -> 
  + 90 deg, 90 pct   -> 1080 deg, 90 pct -> 720 deg, 90 pct  -> 360 deg, 90 pct  ->
 - Step 2
  + 13 cm, 30 pct    -> 10 cm, 20 pct    -> 
  + -5.5 deg, 5 pct  -> -7 deg -> -6 deg -> -5 deg -> -4 pct, 3 pct -> 
  + 75 pct, 2 discs  -> 80 pct -> 70 pct -> 65 pct -> 90 pct -> 95 pct -> 88 pct -> 85 pct -> 72 pct -> 69 pct -> 74 pct -> 73 pct -> 
 - Step 3
  + 4.5 cm, 10 pct   -> 
  + -104 deg, 30 pct -> 
  + 62 cm, 80 pct    -> 
  + 95 pct           -> 
  + 50 cm, 25 pct    -> 
  + 5000 msec        -> 
  + 80 deg, 30 pct   -> 
  + 15 cm, 20 pct    -> 
  + 75 pct, 3 discs  -> 
 - Step 4
  + -95 deg, 30 pct  -> 
  + 50 cm, 40 pct    -> 
  + -110 deg, 30 pct -> 
  + 150 cm, 50 pct   -> 
  + 90 deg, 90 pct   -> 
*/