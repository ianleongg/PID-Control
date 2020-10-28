#include "PID.h"
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  std::cout << "Kp = " << Kp << ", Ki= " << Ki << ", Kd =" << Kd << std::endl;
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PID::UpdateError(double cte) {
 //differential error
  d_error = cte - p_error;
  
  //proportional error
  p_error = cte;
  
  //integral error
  i_error +=  cte;
 
}

double PID::TotalError() {
  
  return -Kp_ * p_error - Kd_ * d_error - Ki_ * i_error; 
}

