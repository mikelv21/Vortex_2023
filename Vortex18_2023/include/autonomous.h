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
const double INDEXER_GO     = 1700;        //pct
const double INDEXER_BACK   = 800;         //ms            
const double WAIT_UNTIL_LAUNCH = 5500;     //ms
const double INTAKE_VEL     = 80;          //pcd 

//-------------------------------------------------------------------------

void shoot_disc(int vel, int t,  int n, int t1, int t2, timeUnits x){
  /* 
  Shoot the disc collected with a given velocity (vel)
  and wait a t time to activate the indexer and make n 
  throws and keep it open a t1 time and t2 close.
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
  and with a given velocity vel
  */
  intake_roller.setVelocity(vel, percent);
  intake_roller.spin(reverse);
  wait(n, t);
  intake_roller.stop();
}


void prueba_autonomo(){
  /* Step 1 */
  // Throw discs
  Drive.driveFor(directionType::rev, 40, distanceUnits::cm, 35, velocityUnits::pct);
  intake_roller.setVelocity(INTAKE_VEL, percent);
  intake_roller.spin(forward);
  Drive.driveFor(directionType::rev, 10, distanceUnits::cm, 30, velocityUnits::pct);
  Drive.turnToHeading(-153, rotationUnits::deg, 10, velocityUnits::pct);  //-145 -> -150 -> -151 -> (battery 100%)
  intake_roller.stop();
  Drive.driveFor(directionType::fwd, 3, distanceUnits::cm, 30, velocityUnits::pct);
  shoot_disc(83, WAIT_UNTIL_LAUNCH, 3, INDEXER_BACK, INDEXER_GO, msec);

  /* Step 2 */
  // Go to the roller
  reset_turnH(-75, 10); //-70 -> -68 -> -72
  Drive.driveFor(directionType::rev, 90, distanceUnits::cm, 35, velocityUnits::pct);
  reset_turnH(37, 5);   //35 -> 38 -> 
  Drive.driveFor(directionType::rev, 32, distanceUnits::cm, 30, velocityUnits::pct);
  activate_intake(INTAKE_VEL, 2000, msec);
}
