#ifndef PID_WITH_TWIDDLE_H
#define PID_WITH_TWIDDLE_H

#include "PID.h"
#include "RunningRMSE.h"

#include <vector>

enum class TwiddleState
{
  kState1,
  kState2,
  kState3,
  kState4,
  kState5,
  kState6,
};

/**
 * The Twiddle class.
 */
class PIDWithTwiddle: public PID {
 public:
  /**
   * Constructor
   */
  PIDWithTwiddle();

  /**
   * Destructor.
   */
  virtual ~PIDWithTwiddle();

  bool startOptimization(int rmse_count, double tolerance, std::vector<double> dp);
  bool IsOptimizationFinished();

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  virtual void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  virtual double TotalError();
 protected:
  /**
   * Twiddle
   */
  std::vector<double> dp;
  double tolerance;
  int rmse_count;
  RunningRMSE running_RMSE;
  double min_error;
  int optimization_index;
  TwiddleState optimization_state;
};

#endif  // PID_WITH_TWIDDLE_H