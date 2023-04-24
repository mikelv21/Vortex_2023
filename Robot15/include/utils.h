/*----------------------------------------------------------------------------*/
/*    Module:       others.h                                                  */
/*    Author:       VORTEX Robotics                                           */
/*                  For more information contact A01705935@tec.mx             */
/*    Created:      18-feb-2023                                               */
/*    Description:  Default header for V5 projects                            */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "robot-config.h"
#include "vex.h"

using namespace vex;

// ---------------------------------------------------------------------------------------------------- 
int spin_flywheel(){
  Flywheel.spin(forward);
  return 0;
}

// ---------------------------------------------------------------------------------------------------- 
void move_to_coordinate(double target_x, double target_y, double target_heading){
  if (target_x == 0 && target_y != 0){
    if (target_y > 0){ Drivetrain.driveFor(fwd, target_y, distanceUnits::cm); }
    if (target_y < 0){ Drivetrain.driveFor(reverse, target_y, distanceUnits::cm); }
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

void shoot_disc(int vel, int t,  int n, int t1, int t2){
  /* Shoot the disc collected with a given velocity (vel) in percentage
   * and wait a t time to activate the indexer and make n throws and 
   * keep it open a t1 time and t2 close. */
  Flywheel.setVelocity(vel, velocityUnits::pct);
  task Fly_task(spin_flywheel, 15);
  task::sleep(t);

  for (int i = 0; i<n; i++){
    Indexer.open();
    wait(t1, msec);
    Indexer.close();
    wait(t2, msec);
  }
  Flywheel.stop(brakeType::hold);
}

void reset_turnH(int d, int vel){
  /* Make a turn re-calibrating inertial sensor */
  DrivetrainInertial.calibrate();
  Drivetrain.turnToHeading(d, deg, vel, velocityUnits::pct);
}

void activate_intake(double rot, double vel){
  /* Activate intake and roller motors to move certain degrees with specific velocity */
  Intake_roller.spinFor(forward, rot, rotationUnits::deg, vel, velocityUnits::pct);
}
