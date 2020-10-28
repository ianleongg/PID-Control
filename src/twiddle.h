#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

class Twiddle {
 private:
  int count;
  int twiddle_phase;
  int param_num;
  double total_cte;
  double best_cte;
  double tolerance;

  std::vector<double> p_params;
  std::vector<double> dp_params;

 public:

  //Constructor
  Twiddle();
 
  //Destructor
  virtual ~Twiddle();

  /**
   * Initialize Twiddle.
   * @param Kp Proportional - to minimize CTE
   * @param Ki Integral - to adjust for steering drift
   * @param Kd Differential - to avoid overshooting
   */
  void Init(double Kp, double Ki, double Kd);

  /**
   * Increment twiddle count and total cross track error
   * @param cte Cross Track Error value to sum
   */
  void IncrementCount(double cte);

  /**
   * @return Current twiddle count (gets reset in updateParams())
   */
  int GetCount();

  /**
   * @return Sum of all the parameter deltas
   */
  double GetTolerance();

  /**
   * Evaluates error (sum(cte) / count) and runs the twiddle algorithm to adjust initial parameter values.
   * @return vector with updated {Kp, Ki, Kd} values
   */
  std::vector<double> UpdateParams();
};

#endif // TWIDDLE_H 