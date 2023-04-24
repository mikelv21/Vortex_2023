#include <iostream>
#include "robot-config.h"
#include "constants.h"

using namespace vex;
extern brain Brain;

//------- Aux function definition -------//
void go_for_roller();
void shot_two_disks();
void go_for_three_stack();
void reset_turnH(double d, double vel);
void shot_three_disks();
void go_for_three_square();

//--------- Main auton function ---------//
void auton(){
  go_for_roller();
  shot_two_disks();
  //go_for_three_stack();
  //shot_three_disks();
  //go_for_three_square();
}

//------- Aux function filling -------//

void go_for_roller(){
  // Move to roller
  Drivetrain.driveFor(directionType::rev, 
                      DISTANCE_FOR_ROLLER, distanceUnits::cm, 
                      VEL_FOR_CHASSIS_ROL, velocityUnits::pct); 
  // Turn roller              
  Intake_roller_group.spinFor(forward,    
                      TURN_FOR_ROLLER, rotationUnits::deg, 
                      VEL_FOR_ROLLER,  velocityUnits::pct);
  // Start spinning Flywheel
  Flywheel.setVelocity(FIRST_DISK_VEL, velocityUnits::pct);
  Flywheel.spin(forward);
}

void shot_two_disks(){
  // Go to throw discs
  Drivetrain.driveFor(directionType::fwd, 
                      DIST_FOR_TWO_DISKS, distanceUnits::cm, 
                      VEL_CHASSIS_TWO_D,  velocityUnits::pct);  
  // Turn to the basket
  Drivetrain.turnToHeading(HEADING_SETPOINT, rotationUnits::deg, 
                            VEL_FOR_HEADING, velocityUnits::pct);              
  // Throw 1st disk
  wait(INDEXER_WAIT*2, msec);
  Indexer.open(); 
  wait(INDEXER_WAIT, msec);
  Indexer.close();
  // Throw 2nd disk
  Flywheel.setVelocity(SECOND_DISK_VEL, velocityUnits::pct);
  wait(INDEXER_WAIT*1.5, msec);
  Indexer.open(); 
  wait(INDEXER_WAIT, msec);
  Indexer.close();
  Flywheel.stop(brakeType::coast);
}

void go_for_three_stack(){
  // Aim to stack
  reset_turnH(TURN_TO_STACK_DEG, TURN_TO_STACK_VEL); 
  // Go for stack
  Drivetrain.driveFor(directionType::rev, 
                      DIST_FOR_STACK, distanceUnits::cm, 
                      VEL_FOR_STACK_CHAS, velocityUnits::pct);
  Drivetrain.driveFor(directionType::fwd, 
                      DIST_FOR_BACK, distanceUnits::cm, 
                      VEL_FOR_STACK_CHAS, velocityUnits::pct);
  // Activate intake to collect the discs
  Intake_roller_group.spin(reverse, VEL_INTAKE_STACK, velocityUnits::pct);  
  // Collecting the discs
  Drivetrain.driveFor(directionType::rev,
                      DIST_FOR_COLLECT_STACK, 
                      distanceUnits::cm, 
                      VEL_COLLECT_STACK, 
                      velocityUnits::pct);  
}

void shot_three_disks(){
  Flywheel.setVelocity(THIRD_DISK_VEL, velocityUnits::pct);
  Flywheel.spin(forward);
  //AQUI LE MOVIMOS ESTABA EN 25 EL STACK VEL
  reset_turnH(TURN_FOR_BAS, TURN_TO_STACK_VEL);   // Aim for the basket
  Intake_roller_group.stop();  

  //1st disk
  wait(WAIT_UNTIL_LAUNCH/2.0, msec);
  Indexer.open(); 
  wait(INDEXER_WAIT, msec);
  Indexer.close();
  //2nd disk
  wait(INDEXER_WAIT, msec);
  Flywheel.setVelocity(FOURHT_DISK_VEL, velocityUnits::pct);
  Indexer.open(); 
  wait(INDEXER_WAIT, msec);
  Indexer.close();
  //3rd disk
  wait(INDEXER_WAIT, msec);
  Flywheel.setVelocity(FIFTH_DISK_VEL, velocityUnits::pct);
  Indexer.open(); 
  wait(INDEXER_WAIT, msec);
  Indexer.close();
  Flywheel.stop(brakeType::coast);
}

void go_for_three_square(){
  reset_turnH(-15, 30); 
  Drivetrain.driveFor(directionType::rev, 
                      60, 
                      distanceUnits::cm, 
                      100, 
                      velocityUnits::pct);
  Drivetrain.stop(hold);
}

void reset_turnH(double d, double vel){
  DrivetrainInertial.calibrate();
  Drivetrain.turnToHeading(d, deg, vel, velocityUnits::pct);
}
