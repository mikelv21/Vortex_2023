/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\aaron                                            */
/*    Created:      Mon Oct 31 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
using namespace vex;
competition Competition;

void pre_auton(void)
{
  vexcodeInit();
  Indexer.close();
  // All activities that occur before the competition starts. Example: clearing encoders, setting servo positions, ...
}

void usercontrol(void)
{
  rc_auto_loop_function_Controller1();
}

int main() {
  pre_auton();
  //auton(); //TODO: AUTON ROUTINES
  usercontrol();
}
