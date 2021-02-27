#ifndef PID_WITH_TWIDDLE_H
#define PID_WITH_TWIDDLE_H

#include "PID.h"
#include "RunningData.h"

#include <vector>

/**
 * The Twiddle State.
 */
enum class TwiddleState
{
  kStateInitial,
  kStateComputeErrorAtInitial,
  kStateComputeErrorWhenIncrease,
  kStateComputeErrorWhenDecrease,
};

/**
 * The PID class with Auto Tuning using Twiddle Algorithm.
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

  /**
   * Start the optimization using Twiddle algorithm.
   * @param rmse_count The number of samples used in computing root mean square eror
   * @param toleranceRatio The finishing tolerance ratio related to the initial step parameters
   * @param dp The twiddle step parameters
   * @output if the start of the optimization is successful
   */
  bool startOptimization(int rmse_count, double toleranceRatio, std::vector<double> dp);

  /**
   * Check if the the optimization is finished.
   * @output if the the optimization is finished
   */
  bool IsOptimizationFinished();

  /**
   * Update the PID error variables given cross track error.
   * It is the implementation of the Twiddle algorithm.
   * @param cte The current cross track error
   */
  virtual void UpdateError(double cte);
  
 protected:
  /**
   * The running twiddle data
   */
  std::vector<double> dp;
  std::vector<double> dpFinished;

  /**
   * The running error data
   */
  int rmse_count;
  RunningData running_RMSE;
  double min_error;

  /**
   * The Twiddle state parameters.
   */
  int optimization_index;
  TwiddleState optimization_state;
};

#endif  // PID_WITH_TWIDDLE_H