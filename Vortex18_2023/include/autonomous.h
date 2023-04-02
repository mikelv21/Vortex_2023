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
const double INTAKE_VEL     = 90;          //pcd 80 -> 90

//-------------------------------------------------------------------------

void shoot_disc(float vel, int t,  int n, int t1, int t2, timeUnits x){
  /* Shoot the disc collected with a given velocity (vel)
   * and wait a t time to activate the indexer and make n 
   * throws and keep it open a t1 time and t2 close. */
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
   * and with a given velocity vel */
  intake_roller.setVelocity(vel, percent);
  intake_roller.spin(reverse);
  wait(n, t);
  intake_roller.stop();
}

void activate_intake2(double vel, int n, timeUnits t, directionType xd){
  /*  Activate intake and roller for a given time n
   * and with a given velocity vel */
  intake_roller.setVelocity(vel, percent);
  intake_roller.spin(xd);
  wait(n, t);
  intake_roller.stop();
}


void prueba_autonomo(){
  /* Step 1 */
  // Throw discs
  Drive.driveFor(directionType::rev, 40, distanceUnits::cm, 35, velocityUnits::pct);
  intake_roller.setVelocity(INTAKE_VEL, percent);
  intake_roller.spin(forward);
  Drive.driveFor(directionType::rev, 8, distanceUnits::cm, 30, velocityUnits::pct);
  wait(2800, msec);
  intake_roller.stop();
  Drive.turnToHeading(-154, rotationUnits::deg, 10, velocityUnits::pct);  //-145 -> -150 -> -151 -> -153 -> -155
  Drive.driveFor(directionType::fwd, 3, distanceUnits::cm, 30, velocityUnits::pct);
  shoot_disc(70.5, WAIT_UNTIL_LAUNCH, 3, INDEXER_BACK, INDEXER_GO, msec); //83 -> 81 -> 79 -> 75 -> 72

  /* Step 2 */
  // Go to the roller
  Drive.driveFor(directionType::rev, 40, distanceUnits::cm, 30, velocityUnits::pct);
  reset_turnH(-90, 10);
  Drive.driveFor(directionType::rev, 95, distanceUnits::cm, 40, velocityUnits::pct); // 90 -> 
  reset_turnH(85, 10); // 100 -> 80 -> 85 -> 
  Drive.driveFor(directionType::rev, 20, distanceUnits::cm, 30, velocityUnits::pct); // 15 -> 
  activate_intake(INTAKE_VEL, 1000, msec);

  //reset_turnH(-75, 10); //-70 -> -68 -> -72
  //Drive.driveFor(directionType::rev, 95, distanceUnits::cm, 40, velocityUnits::pct); // 90 ->  
  //reset_turnH(47, 5);   //35 -> 38 -> 37 -> 40 -> 45 ->  
  //Drive.driveFor(directionType::rev, 27, distanceUnits::cm, 30, velocityUnits::pct); // 32 -> 25 -> 21 -> 23 -> 25
  //activate_intake(INTAKE_VEL, 1000, msec);
}


void skills(){
  /* Step 1 */
  // Throw discs
  Drive.driveFor(directionType::rev, 40, distanceUnits::cm, 35, velocityUnits::pct);
  intake_roller.setVelocity(INTAKE_VEL, percent);
  intake_roller.spin(forward);
  Drive.driveFor(directionType::rev, 8, distanceUnits::cm, 30, velocityUnits::pct);
  wait(2800, msec);
  intake_roller.stop();
  Drive.turnToHeading(-154, rotationUnits::deg, 20, velocityUnits::pct);  //-145 -> -150 -> -151 -> -153 -> -155
  Drive.driveFor(directionType::fwd, 3, distanceUnits::cm, 30, velocityUnits::pct);
  shoot_disc(70.5, WAIT_UNTIL_LAUNCH, 3, INDEXER_BACK, INDEXER_GO, msec); //83 -> 81 -> 79 -> 75 -> 72

  /* Step 2 */
  // Go to the roller
  Drive.driveFor(directionType::rev, 40, distanceUnits::cm, 30, velocityUnits::pct);
  reset_turnH(-90, 20);
  Drive.driveFor(directionType::rev, 95, distanceUnits::cm, 40, velocityUnits::pct); // 90 -> 
  reset_turnH(85, 20); // 100 -> 80 -> 85 -> 
  Drive.driveFor(directionType::rev, 19, distanceUnits::cm, 30, velocityUnits::pct); // 15 -> 
  activate_intake2(INTAKE_VEL, 2000, msec, reverse);

  /* Step 3 */
  // Go to the roller 2 
  Drive.driveFor(directionType::fwd, 50, distanceUnits::cm, 30, velocityUnits::pct);
  reset_turnH(-100, 20);
  Drive.driveFor(directionType::rev, 73, distanceUnits::cm, 30, velocityUnits::pct); // 70 -> 
  activate_intake2(INTAKE_VEL, 1000, msec, reverse);

}