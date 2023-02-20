/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VORTEX Robotics                                           */
/*                  For more information contact A01706424@tec.mx             */
/*    Created:      18-feb-2023                                               */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "autonomous.h"

using namespace vex;
competition Competition;

// All activities that occur before the competition starts. Example: clearing encoders, setting servo positions, ...
void pre_auton(void)
{
  vexcodeInit();
}

void auton(void){
  //  autonomous_time(20, 20, 0);
  prueba_autonomo();
}

void usercontrol(void)
{
  rc_auto_loop_function_Controller1();
}

int main() {
  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);
  
  pre_auton();
}
