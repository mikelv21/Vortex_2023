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
const int    DEADBAND       = 10;          //pct
const double INDEXER_GO     = 1500;        //pct
const double INDEXER_BACK   = 400;         //ms            
const double WAIT_UNTIL_LAUNCH = 3100;     //ms
const double FLYWHEEL_VEL   = 69;          //pct
const double INTAKE_VEL     = 70;          //pcd 

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

void activate_intake(double vel, int n, timeUnits t){
  /* 
  Activate intake and roller for a given time n
  and with a given velocity vel in percentage.
  */
  intake_roller.setVelocity(vel, percent);
  intake_roller.spin(forward);
  wait(n, t);
  intake_roller.stop();
}

void prueba_autonomo(){
  /* Step 1 */
  // Go to the roller
  Drive.driveFor(directionType::rev, 5, distanceUnits::cm, 70, velocityUnits::pct);
  // Move the roller
  activate_intake(90, 1000, msec);

  /* Step 2 */
  // Go to throw discs
  Drive.driveFor(directionType::fwd, 30, distanceUnits::cm, 70, velocityUnits::pct);
  // Turn to the basket
  Drive.turnToHeading(2, deg, 70, velocityUnits::pct);
  // Shoot the discs
  shoot_disc(FLYWHEEL_VEL, WAIT_UNTIL_LAUNCH, 2, INDEXER_BACK, INDEXER_GO, msec);

  /* Step 3 */
  // ...
}