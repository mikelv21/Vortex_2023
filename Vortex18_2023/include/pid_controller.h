/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       pid_controller.h                                          */
/*    Author:       VORTEX Robotics                                           */
/*                  For more information contact A01706424@tec.mx             */
/*    Created:      18-feb-2023                                               */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

class PIDController 
{
  private:
    double kp_;
    double ki_;
    double kd_;
    double maxOutput_;
    double lastError_ = 0;
    double integral_ = 0;
    
  public:
    PIDController(double kp, double ki, double kd, double maxOutput):
      kp_(kp), 
      ki_(ki), 
      kd_(kd), 
      maxOutput_(maxOutput) {}

    double update(double error, double dt) 
    {
      double output = kp_ * error + ki_ * integral_ + kd_ * (error - lastError_) / dt;
      lastError_ = error;
      integral_ += error * dt;
      if      (output > maxOutput_)  {output = maxOutput_;} 
      else if (output < -maxOutput_) {output = -maxOutput_;}
      return output;
    }
};