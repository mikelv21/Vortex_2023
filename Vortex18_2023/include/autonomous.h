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
void prueba_autonomo(){   
  // Step 1: Go for a disc
  Drive.turnToHeading(180, deg, 70, velocityUnits::pct);
  Drive.driveFor(directionType::fwd, 5, distanceUnits::cm, 80, velocityUnits::pct);
  intake_roller.setVelocity(INTAKE_VEL, percent);
  intake_roller.spin(forward);
  wait(2000, timeUnits::msec);
  intake_roller.stop();
  Drive.driveFor(directionType::fwd, 20, distanceUnits::cm, 80, velocityUnits::pct);

  // Step 2: Throw discs
  Flywheel.spin(forward, FLYWHEEL_VEL, velocityUnits::pct);
  wait(WAIT_UNTIL_LAUNCH, msec);
  for (int i = 0; i<3; i++){
    Indexer.open();
    wait(INDEXER_BACK, msec);
    Indexer.close();
    wait(INDEXER_GO, msec);
  }
  Flywheel.stop();
  
}