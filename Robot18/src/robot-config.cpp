#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
// Controller
controller Controller1 = controller(primary);

// Drivetrain
motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT2, ratio18_1, true);
motor leftMotorC = motor(PORT3, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT4, ratio18_1, true);
motor rightMotorB = motor(PORT5, ratio18_1, false);
motor rightMotorC = motor(PORT6, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
inertial InertialSensor = inertial(PORT16);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, InertialSensor, 319.19, 320, 40, mm, 1.3125);

// Flywheel
motor FlywheelMotorA = motor(PORT8, ratio18_1, false);
motor FlywheelMotorB = motor(PORT9, ratio18_1, false);
motor_group Flywheel = motor_group(FlywheelMotorA, FlywheelMotorB);

// Indexer
digital_out Indexer = digital_out(Brain.ThreeWirePort.A);

// Intake & Roller
motor Intake_roller = motor(PORT7, ratio18_1, false);

// Expansor
motor Expansor = motor(PORT11, ratio18_1, false);
