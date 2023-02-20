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
#include "position_control.h"
#include "pid_controller.h"
using namespace vex;

// Set up devices and constants
extern brain Brain;
inertial inertialSensor(PORT16);
const double WHEEL_TRAVEL   = 84*M_PI;  // mm
const double MAX_SPEED      = 50;       // in percent
const double MAX_DIST_ERROR = 1;        // in cm
const double MAX_ANG_ERROT  = 1;        // in degrees 
const double DT             = 0.02;     // in seconds
const double KP             = 0.1;
const double KI             = 0.01;
const double KD             = 0.001;

void autonomous_time(double targetX, double targetY, double targetHeading) //TODO target heading
{
  // Set the robot's parameters
  double x               = 0;       // in centimeters
  double y               = 0;       // in centimeters
  double heading_angle   = 0;       // in degrees
  
  LeftBackMotor.setStopping(hold);
  RightBackMotor.setStopping(hold);

  // PID controller for both motors
  PIDController leftPID(KP, KI, KD, MAX_SPEED);
  PIDController rightPID(KP, KI, KD, MAX_SPEED);

  while (distanceToTarget(x, y, targetX, targetY) > 1) 
  {
    double angleDifference = angleToTarget(x, y, targetX, targetY, heading_angle);

    double rotationSpeed = leftPID.update(angleDifference, DT);
    double leftSpeed = MAX_SPEED - rotationSpeed;
    double rightSpeed = MAX_SPEED + rotationSpeed;
    
    double leftPosition = LeftBackMotor.position(degrees);
    double rightPosition = RightBackMotor.position(degrees);
    
    double leftError = leftPosition - rightPosition;
    double rightError = rightPosition - leftPosition;
    
    double leftOutput = leftPID.update(leftError, DT);
    double rightOutput = rightPID.update(rightError, DT);
    
    LeftBackMotor.spin(forward, leftSpeed + leftOutput, percent);
    RightBackMotor.spin(forward, rightSpeed + rightOutput, percent);

    // Update the robot's position and heading based on the motor encoders and inertial sensor
    double leftDistance = (LeftBackMotor.position(degrees) / 360) * WHEEL_TRAVEL;
    double rightDistance = (RightBackMotor.position(degrees) / 360) * WHEEL_TRAVEL;
    double totalDistance = (leftDistance + rightDistance) / 2;
    x += totalDistance * std::cos(heading_angle * M_PI / 180);
    y += totalDistance * std::sin(heading_angle * M_PI / 180);
    heading_angle = inertialSensor.heading();
    wait(DT, sec);
  }

  // Stop the motors and wait for a moment
  LeftBackMotor.stop();
  RightBackMotor.stop();
  wait(500, msec);
  LeftBackMotor.setStopping(hold);
  RightBackMotor.setStopping(hold);
  Brain.Screen.clearScreen(color::green);
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Target reached!");
  // Spin the motors to hold the robot in place
  LeftBackMotor.spin(forward);
  RightBackMotor.spin(forward);
  while (true) {
    wait(100, msec);
  }

}

void prueba_autonomo()
{
    LeftMotors.spin(forward, 20, percent);
    RightMotors.spin(forward, 20, percent);
    wait(2000, msec);
    LeftMotors.stop();
    RightMotors.stop();
}