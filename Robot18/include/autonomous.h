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
#include "utils.h"
#include "robot-config.h"
using namespace vex;

void auton(){
  /* Step 1 */
  // Throw discs
  move_to_coordinate(0, -40, 0);
  Intake_roller.spin(forward, INTAKE_VEL, pct);
  move_to_coordinate(0, -8, 0);
  wait(2000, msec);
  Intake_roller.stop();
  move_to_coordinate(0, 3, 0);
  shoot_disc(70.5, WAIT_UNTIL_LAUNCH, 3, INDEXER_BACK, INDEXER_GO, msec);

  /* Step 2
  // Go to the roller
  Drivetrain.driveFor(directionType::rev, 40, distanceUnits::cm, 30, velocityUnits::pct);
  reset_turnH(-90, 10);
  Drivetrain.driveFor(directionType::rev, 95, distanceUnits::cm, 40, velocityUnits::pct); // 90 -> 
  reset_turnH(85, 10); // 100 -> 80 -> 85 -> 
  Drivetrain.driveFor(directionType::rev, 20, distanceUnits::cm, 30, velocityUnits::pct); // 15 -> 
  activate_intake(INTAKE_VEL, 1000, msec);  */

  //reset_turnH(-75, 10); 
  //Drive.driveFor(directionType::rev, 95, distanceUnits::cm, 40, velocityUnits::pct); // 90 ->  
  //reset_turnH(47, 5);
  //Drive.driveFor(directionType::rev, 27, distanceUnits::cm, 30, velocityUnits::pct); // 32 -> 25 -> 21 -> 23 -> 25
  //activate_intake(INTAKE_VEL, 1000, msec);
}


void skills(){
  /* Step 1 */
  // Throw discs
  Drivetrain.driveFor(directionType::rev, 40, distanceUnits::cm, 35, velocityUnits::pct);
  Intake_roller.setVelocity(INTAKE_VEL, percent);
  Intake_roller.spin(forward);
  Drivetrain.driveFor(directionType::rev, 8, distanceUnits::cm, 30, velocityUnits::pct);
  wait(2800, msec);
  Intake_roller.stop();
  Drivetrain.turnToHeading(-154, rotationUnits::deg, 20, velocityUnits::pct); 
  Drivetrain.driveFor(directionType::fwd, 3, distanceUnits::cm, 30, velocityUnits::pct);
  shoot_disc(70.5, WAIT_UNTIL_LAUNCH, 3, INDEXER_BACK, INDEXER_GO, msec);

  /* Step 2 */
  // Go to the roller
  Drivetrain.driveFor(directionType::rev, 40, distanceUnits::cm, 30, velocityUnits::pct);
  reset_turnH(-90, 20);
  Drivetrain.driveFor(directionType::rev, 95, distanceUnits::cm, 40, velocityUnits::pct); 
  reset_turnH(85, 20);
  Drivetrain.driveFor(directionType::rev, 19, distanceUnits::cm, 30, velocityUnits::pct);
  activate_intake2(INTAKE_VEL, 2000, msec, reverse);

  /* Step 3 */
  // Go to the roller 2 
  Drivetrain.driveFor(directionType::fwd, 50, distanceUnits::cm, 30, velocityUnits::pct);
  reset_turnH(-100, 20);
  Drivetrain.driveFor(directionType::rev, 73, distanceUnits::cm, 30, velocityUnits::pct);
  activate_intake2(INTAKE_VEL, 1000, msec, reverse);

}

/* Record of the values
 - Step 1
 - Step 2 
 - Step 3
*/