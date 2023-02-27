/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\aaron                                            */
/*    Created:      Mon Oct 31 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "autonomous.h"

using namespace vex;
competition Competition;

void display_info_static(){
  // Display Left Motors
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Left Motors");
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("-------------");
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("LFM Temp: ");
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("LMM Temp: ");
  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("LBM Temp: ");

  // Display Right Motors
  Brain.Screen.setCursor(1, 25);
  Brain.Screen.print("Right Motors");
  Brain.Screen.setCursor(2, 25);
  Brain.Screen.print("-------------");
  Brain.Screen.setCursor(3, 25);
  Brain.Screen.print("RFM Temp: ");
  Brain.Screen.setCursor(4, 25);
  Brain.Screen.print("RMM Temp: ");
  Brain.Screen.setCursor(5, 25);
  Brain.Screen.print("RBM Temp: ");

  // Display heading (yaw angle as 0-360 deg)
  Brain.Screen.setCursor(7, 1);
  Brain.Screen.print("Heading: ");
}

void display_info_dinamic(){
  Brain.Screen.setCursor(3, 10);
  Brain.Screen.print(LeftFrontMotor.temperature(celsius));
  Brain.Screen.setCursor(4, 10);
  Brain.Screen.print(LeftMiddleMotor.temperature(celsius));
  Brain.Screen.setCursor(5, 10);
  Brain.Screen.print(LeftBackMotor.temperature(celsius));

  Brain.Screen.setCursor(3, 36);
  Brain.Screen.print(RightFrontMotor.temperature(celsius));
  Brain.Screen.setCursor(4, 36);
  Brain.Screen.print(RightMiddleMotor.temperature(celsius));
  Brain.Screen.setCursor(5, 36);
  Brain.Screen.print(RightBackMotor.temperature(celsius));

  Brain.Screen.setCursor(7, 10);
  Brain.Screen.print(inertialSensor.heading());
}

// All activities that occur before the competition starts. Example: clearing encoders, setting servo positions, ...
void pre_auton(void){
  vexcodeInit();
  Indexer.close();
}

void auton(void){
  prueba_autonomo();
}

void usercontrol(void){
  rc_auto_loop_function_Controller1();
}

int main() {
  pre_auton();
  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);

  display_info_static();  

  while(true){
    display_info_dinamic();
  }
}
