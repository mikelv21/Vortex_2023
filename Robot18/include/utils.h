#include "vex.h"
#include "robot-config.h"

using namespace vex;

// Constants 
const int    DEADBAND       = 10;          //pct
const double INDEXER_GO     = 1700;        //pct
const double INDEXER_BACK   = 800;         //ms            
const double WAIT_UNTIL_LAUNCH = 5500;     //ms
const double INTAKE_VEL     = 90;          //pcd 80 -> 90

//-------------------------------------------------------------------------
int spin_flywheel(){
  Flywheel.spin(forward);
  return 0;
}

//-------------------------------------------------------------------------
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
  InertialSensor.resetHeading();
  if (target_heading != 0){
    Drivetrain.turnToHeading(target_heading, rotationUnits::deg);
  }
  Drivetrain.stop(brakeType::hold);
}

void shoot_disc(float vel, int t,  int n, int t1, int t2, timeUnits x){
  /* Shoot the disc collected with a given velocity (vel)
   * and wait a t time to activate the indexer and make n 
   * throws and keep it open a t1 time and t2 close. */
   Flywheel.setVelocity(vel, velocityUnits::pct);
  task Fly_task(spin_flywheel, 15);
  task::sleep(t);

  wait(t, x);
  for (int i = 0; i<n; i++){
    Indexer.set(true);
    wait(t1, x);
    Indexer.set(false);
    wait(t2, x);
  }
  Flywheel.stop(brakeType::hold);
}

void reset_turnH(int d, int vel){
  /* Make a turn re-calibrating inertial sensor */
  InertialSensor.calibrate();
  Drivetrain.turnToHeading(d, deg, vel, velocityUnits::pct);
}

void activate_intake(double vel, int n, timeUnits t){
  /* Activate intake and roller for a given time n and with a given velocity vel */
  Intake_roller.setVelocity(vel, percent);
  Intake_roller.spin(reverse);
  wait(n, t);
  Intake_roller.stop();
}

void activate_intake2(double vel, int n, timeUnits t, directionType xd){
  /*  Activate intake and roller for a given time n and with a given velocity vel */
  Intake_roller.setVelocity(vel, percent);
  Intake_roller.spin(xd);
  wait(n, t);
  Intake_roller.stop();
}
