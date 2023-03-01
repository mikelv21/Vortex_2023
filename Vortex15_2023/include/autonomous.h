/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       autonomous.h                                              */
/*    Author:       VORTEX Robotics                                           */
/*                  For more information contact A01706424@tec.mx             */
/*    Created:      18-feb-2023                                               */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <cmath>
#include "robot-config.h"
using namespace vex;

// Set up devices and constants
extern brain Brain;

/************ Constants ****************/
const int    DEADBAND       = 10;         //pct
const double INDEXER_GO     = 1700;       //ms
const double INDEXER_BACK   = 1000;       //ms            
const double WAIT_UNTIL_LAUNCH = 6000;    //ms
const double INTAKE_VEL     = 70;         //pcd 

//-------------------------------------------------------------------------
void shoot_disc(int vel, int t,  int n, int t1, int t2, timeUnits x){
  /* 
  Shoot the disc collected with a given velocity (vel) in percentage
  and wait a t time to activate the indexer and make n throws and 
  keep it open a t1 time and t2 close.
  */
  Flywheel.spin(forward, vel, velocityUnits::pct);
  wait(t, x);
  for (int i = 0; i<n; i++){
    Indexer.open();
    wait(t1, x);
    Indexer.close();
    wait(t2, x);
  }
  Flywheel.stop();
}

void reset_turnH(int d, int vel){
  inertialSensor.calibrate();
  Drive.turnToHeading(d, deg, vel, velocityUnits::pct);
}

void activate_intake(double vel, int n, timeUnits t){
  /* 
  Activate intake and roller for a given time n
  and with a given velocity vel in percentage.
  */
  intake_roller.setVelocity(vel, percent);
  intake_roller.spin(reverse);
  wait(n, t);
  intake_roller.stop();
}

void prueba_autonomo(){
  /* Step 1 */
  // Go to the roller
  Drive.driveFor(directionType::rev, 5, distanceUnits::cm, 20, velocityUnits::pct);
  // Move the roller
  activate_intake(90, 1700, msec);  // 1000 -> 1500 -> 

  /* Step 2 */
  // Go to throw discs
  Drive.driveFor(directionType::fwd, 10, distanceUnits::cm, 20, velocityUnits::pct);
  // Turn to the basket
  Drive.turnToHeading(-5, rotationUnits::deg, 5, velocityUnits::pct); // -11 -> 
  // Shoot the discs
  shoot_disc(100, WAIT_UNTIL_LAUNCH, 2, INDEXER_BACK, INDEXER_GO, msec);  // 100 -> 90 -> 92 -> 

  /* Step 3 */
  // Go for the rest of discs
  Drive.driveFor(directionType::rev, 3, distanceUnits::cm, 10, velocityUnits::pct);
  reset_turnH(-105, 10); // -101 -> 
  Drive.driveFor(directionType::rev, 50, distanceUnits::cm, 90, velocityUnits::pct); // 20 -> 50 
  Drive.driveFor(directionType::fwd, 5, distanceUnits::cm, 20, velocityUnits::pct);
  intake_roller.setVelocity(100, percent);
  intake_roller.spin(forward);
  Drive.driveFor(directionType::rev, 40, distanceUnits::cm, 30, velocityUnits::pct); // 60 -> 30 -> 
  wait(6500, msec);
  intake_roller.stop();
  reset_turnH(80, 10);  // 60 -> 75 -> 90 -> 110 -> 
  Drive.driveFor(directionType::fwd, 15, distanceUnits::cm, 20, velocityUnits::pct);
  // Shoot the discs
  shoot_disc(95, WAIT_UNTIL_LAUNCH, 3, INDEXER_BACK, INDEXER_GO, msec); // 100 -> 90 -> 
}