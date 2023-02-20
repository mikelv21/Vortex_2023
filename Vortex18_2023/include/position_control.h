/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       position_control.h                                        */
/*    Author:       VORTEX Robotics                                           */
/*                  For more information contact A01706424@tec.mx             */
/*    Created:      18-feb-2023                                               */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <cmath>
using namespace vex;

double distanceToTarget(double x, double y, double targetX, double targetY) 
{
  double dx = targetX - x;
  double dy = targetY - y;
  return std::sqrt(dx*dx + dy*dy);
}

double angleToTarget(double x, double y, double targetX, double targetY, double heading_angle) 
{
  double dx = targetX - x;
  double dy = targetY - y;
  double targetAngle = std::atan2(dy, dx) * 180 / M_PI;
  double angleDifference = targetAngle - heading_angle;
  while (angleDifference > 180)  {angleDifference -= 360;}
  while (angleDifference < -180) {angleDifference += 360;}
  return angleDifference;
}