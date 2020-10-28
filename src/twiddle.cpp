#include "twiddle.h"
#include <cmath>

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(double Kp, double Ki, double Kd){
  p_params = {Kp, Ki, Kd};
  //dp_params = {1.0, 1.0, 1.0};
  dp_params = {0.1*Kp, 0.1*Ki, 0.1*Kd};
  count = 0;
  twiddle_phase = 0; 
  param_num = 0;
  total_cte = 0.0;
  best_cte = 1.0;
  tolerance = 0.0;
}

void Twiddle::IncrementCount(double cte) {
  total_cte += fabs(cte);
  count++;
}

int Twiddle::GetCount() {
  return count;
}

double Twiddle::GetTolerance() {
  return tolerance;
}

std::vector<double> Twiddle::UpdateParams() {
  double curr_cte = total_cte / count;
  count = 0;
  total_cte = 0;
  tolerance = dp_params[0] + dp_params[1] + dp_params[2];

  bool update_params = false;
  
  if (twiddle_phase == 0) {
    p_params[param_num] += dp_params[param_num];
    twiddle_phase = 1;
  } 
  
  else if (twiddle_phase == 1) {
    
    if (curr_cte < best_cte) {
      best_cte = curr_cte;
      dp_params[param_num] *= 1.1;
      twiddle_phase = 0;
      update_params = true;
    }
    
    else {
      p_params[param_num] -= 2 * dp_params[param_num];
      twiddle_phase = 2;
    }
  } 
  
  else if (twiddle_phase == 2) {
   
    if (curr_cte < best_cte) {
      best_cte = curr_cte;
      dp_params[param_num] *= 1.1;
    } 
    
    else {
      p_params[param_num] += dp_params[param_num];
      dp_params[param_num] *= 0.9;
    }
    
    twiddle_phase = 0;
    update_params = true;
  }
  
  if (update_params && (++param_num == 3)) {
    param_num = 0;
  }
  
  return p_params;
}
  