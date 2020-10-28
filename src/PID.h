#ifndef PID_H
#define PID_H

class PID {
 
 public:
  //Constructor 
  PID();
  
  // Destructor.
  virtual ~PID();

  /**
   * Initialize PID.
   * @param Kp Proportional - to minimize CTE
   * @param Ki Integral - to adjust for steering drift
   * @param Kd Differential - to avoid overshooting
   */
  void Init(double Kp, double Ki, double Kd);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 private:
  //PID Errors
  double p_error;
  double i_error;
  double d_error;
 

  //PID Coefficients
  double Kp_;
  double Ki_;
  double Kd_;
};

#endif  // PID_H