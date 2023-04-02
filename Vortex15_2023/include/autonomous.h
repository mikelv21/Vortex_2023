/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       autonomous.h                                              */
/*    Author:       VORTEX Robotics                                           */
/*                  For more information contact A01706424@tec.mx             */
/*    Created:      18-feb-2023                                               */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

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
  /* Shoot the disc collected with a given velocity (vel) in percentage
   * and wait a t time to activate the indexer and make n throws and 
   * keep it open a t1 time and t2 close. */
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
  /* Activate intake and roller for a given time n
   * and with a given velocity vel in percentage. */
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
  Drive.driveFor(directionType::fwd, 13, distanceUnits::cm, 20, velocityUnits::pct);  // 10 -> 
  // Turn to the basket
  Drive.turnToHeading(-5.5, rotationUnits::deg, 5, velocityUnits::pct); // -11 -> -5 -> 
  // Shoot the discs
  shoot_disc(95, WAIT_UNTIL_LAUNCH, 2, INDEXER_BACK, INDEXER_GO, msec);  // 100 -> 90 -> 92 -> 

  /* Step 3 */
  // Go for the rest of discs
  Drive.driveFor(directionType::rev, 4.5, distanceUnits::cm, 10, velocityUnits::pct);  // 3 -> 5 -> 
  reset_turnH(-104, 10); // -101 -> -105 -> 
  Drive.driveFor(directionType::rev, 52, distanceUnits::cm, 78, velocityUnits::pct); // 20 -> 50 -> 85 -> 80 -> 75 ->
  Drive.driveFor(directionType::fwd, 10, distanceUnits::cm, 30, velocityUnits::pct); // 5 -> 
  intake_roller.setVelocity(95, percent);
  intake_roller.spin(forward);
  Drive.driveFor(directionType::rev, 50, distanceUnits::cm, 25, velocityUnits::pct); // 60 -> 30 -> 40 -> 
  wait(7000, msec);
  intake_roller.stop();
  reset_turnH(80, 10);  // 60 -> 75 -> 90 -> 110 -> 
  Drive.driveFor(directionType::fwd, 15, distanceUnits::cm, 20, velocityUnits::pct); // 
  // Shoot the discs
  shoot_disc(97, WAIT_UNTIL_LAUNCH, 3, INDEXER_BACK, INDEXER_GO, msec); // 100 -> 90 -> 
}


void skills(){
  /* Step 1 */
  // Go to the roller
  Drive.driveFor(directionType::rev, 3, distanceUnits::cm, 20, velocityUnits::pct);
  // Move the roller
  activate_intake(90, 2000, msec);  // 1000 -> 1500 -> 

  /* Step 2 */
  // Go to throw discs
  Drive.driveFor(directionType::fwd, 13, distanceUnits::cm, 20, velocityUnits::pct);  // 10 -> 
  // Turn to the basket
  Drive.turnToHeading(-5.5, rotationUnits::deg, 10, velocityUnits::pct); // -11 -> -5 -> 
  // Shoot the discs
  shoot_disc(90, WAIT_UNTIL_LAUNCH, 2, INDEXER_BACK, INDEXER_GO, msec);  // 100 -> 90 -> 92 -> 90 -> 

  /* Step 3 */
  // Go for the rest of discs
  Drive.driveFor(directionType::rev, 4.5, distanceUnits::cm, 30, velocityUnits::pct);  // 3 -> 5 -> 
  reset_turnH(-104, 10); // -101 -> -105 -> 
  Drive.driveFor(directionType::rev, 52, distanceUnits::cm, 78, velocityUnits::pct); // 20 -> 50 -> 85 -> 80 -> 75 ->
  Drive.driveFor(directionType::fwd, 10, distanceUnits::cm, 30, velocityUnits::pct); // 5 -> 
  intake_roller.setVelocity(95, percent);
  intake_roller.spin(forward);
  Drive.driveFor(directionType::rev, 50, distanceUnits::cm, 40, velocityUnits::pct); // 60 -> 30 -> 40 -> 
  wait(7000, msec);
  intake_roller.stop();
  reset_turnH(80, 20);  // 60 -> 75 -> 90 -> 110 -> 
  Drive.driveFor(directionType::fwd, 15, distanceUnits::cm, 30, velocityUnits::pct); // 
  // Shoot the discs
  shoot_disc(95, WAIT_UNTIL_LAUNCH, 3, INDEXER_BACK, INDEXER_GO, msec); // 100 -> 90 -> 97 -> 

  /*Step 4 */
  // Go for roller 2
  reset_turnH(-95, 20); 
  Drive.driveFor(directionType::fwd, 50, distanceUnits::cm, 40, velocityUnits::pct); // 
  reset_turnH(-110, 20); 
  Drive.driveFor(directionType::rev, 150, distanceUnits::cm, 50, velocityUnits::pct); // 
  activate_intake(90, 1700, msec);  // 1000 -> 1500 -> 

}