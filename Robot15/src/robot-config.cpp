#include "vex.h"
#include "vex_task.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// Constants
double INTAKE_VEL   = 95;     // pcd: 70  -> 85 -> 
double FLYWHEEL_VEL = 10000;  // mv: 10000 -> 
double EXPANSOR_DEG = -90;    // deg: -50 -> 70 -> 

// VEXcode device constructors
// Set controller
controller Controller1 = controller(primary);

// Set Smartdrive
motor LeftFrontMotor = motor(PORT1, ratio18_1, true);
motor LeftMiddleMotor = motor(PORT2, ratio18_1, false);
motor LeftBackMotor = motor(PORT3, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(LeftFrontMotor, LeftMiddleMotor, LeftBackMotor);
motor RightFrontMotor = motor(PORT6, ratio18_1, false);
motor RightMiddleMotor = motor(PORT5, ratio18_1, true);
motor RightBackMotor = motor(PORT4, ratio18_1, false);
motor_group RightDriveSmart = motor_group(RightFrontMotor, RightMiddleMotor, RightBackMotor);
inertial DrivetrainInertial = inertial(PORT17);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 299.24, 320, 40, mm, 0.5);

// Set Flywheel
motor FlywheelDown = motor(PORT18, ratio18_1, true);
motor FlywheelUp = motor(PORT19, ratio18_1, true);
motor_group Flywheel = motor_group(FlywheelDown, FlywheelUp);

// Set Expansor
motor Expansor = motor(PORT11, ratio18_1, false);

// Set Intake and roller
// Intake-roller
motor Intake_roller_A = motor(PORT7, ratio18_1, false);
motor Intake_roller_B = motor(PORT12, ratio18_1, true);
motor_group Intake_roller = motor_group(Intake_roller_A, Intake_roller_B);

// Set pneumatic indexer
pneumatics Indexer = pneumatics(Brain.ThreeWirePort.H);

// VEXcode generated functions
int threshold = 10;

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;


bool isFlywheelRunning = false;
int Flywheeltask(){
  while(true){
    if(isFlywheelRunning){
      Flywheel.spin(fwd, FLYWHEEL_VEL, voltageUnits::mV);
    }
    task::sleep(10);
  }
}

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  task Fly_task(Flywheeltask, 15);
  // process the controller input every 20 milliseconds & update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // DRIVETRIAN
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis1 | right = Axis3 - Axis1
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
      if (drivetrainLeftSideSpeed < threshold && drivetrainLeftSideSpeed > -threshold) {  // check if the value is inside of the deadband range
        if (DrivetrainLNeedsToBeStopped_Controller1) {      // check if the left motor has already been stopped
          LeftDriveSmart.stop();                            // stop the left drive motor
          DrivetrainLNeedsToBeStopped_Controller1 = false;  // tell the code that the left motor has been stopped
        }
      } 
      // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
      else { DrivetrainLNeedsToBeStopped_Controller1 = true; }
      if (drivetrainRightSideSpeed < threshold && drivetrainRightSideSpeed > -threshold) {  // check if the value is inside of the deadband range
        if (DrivetrainRNeedsToBeStopped_Controller1) {      // check if the right motor has already been stopped
          RightDriveSmart.stop();                           // stop the right drive motor
          DrivetrainRNeedsToBeStopped_Controller1 = false;  // tell the code that the right motor has been stopped
        }
      }
      // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range 
      else { DrivetrainRNeedsToBeStopped_Controller1 = true; }
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) { LeftDriveSmart.spin(forward, drivetrainLeftSideSpeed, percent); }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) { RightDriveSmart.spin(forward, drivetrainRightSideSpeed, percent); }

      // FLYWHEEL
      // check the ButtonL2 status to control Flywheel
      if (Controller1.ButtonL2.pressing() && isFlywheelRunning == false) {
        isFlywheelRunning = true;
        wait(400, msec);
      } 
      else if (Controller1.ButtonL2.pressing() && isFlywheelRunning == true) {
        isFlywheelRunning = false;
        Flywheel.stop(brakeType::coast);
        wait(400, msec);
      }

      // INDEXER
      // check the ButtonR2 status to control indexer
      if (Controller1.ButtonR2.pressing()) { Indexer.set(true); }
      else { Indexer.set(false); }

      // INTAKE & ROLLER
      // check the ButtonA/ButtonB status to control Intake_roller
      if (Controller1.ButtonA.pressing()) { Intake_roller.spin(forward, INTAKE_VEL, percent); } 
      else if (Controller1.ButtonB.pressing()) { Intake_roller.spin(reverse, INTAKE_VEL, percent); } 
      else { Intake_roller.stop(); }

      // EXPANSOR
      // check the ButtonX status to control Expansor
      if (Controller1.ButtonX.pressing()) { Expansor.spinToPosition(EXPANSOR_DEG, rotationUnits::deg, 50, velocityUnits::pct); } 
      else { Expansor.stop(brakeType::hold); }
    }
    wait(20, msec);  // wait before repeating the process
  }
  return 0;
}


/* Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * This should be called at the start of your int main function. */
void vexcodeInit( void ) {
  DrivetrainInertial.calibrate();
  while (DrivetrainInertial.isCalibrating()) { wait(25, msec); }
  wait(50, msec);
  Brain.Screen.clearScreen();
}
