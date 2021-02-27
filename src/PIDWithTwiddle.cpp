#include "PIDWithTwiddle.h"

#include <iostream>
#include <vector>


PIDWithTwiddle::PIDWithTwiddle() : PID() {
  /**
   * Constructor
   */
  min_error = 99999999;
  optimization_index = 0;
  optimization_state = TwiddleState::kStateComputeErrorAtInitial;
  dpFinished = {100, 100, 100};
  dp = {0, 0, 0};
}

PIDWithTwiddle::~PIDWithTwiddle() {
  /**
   * Destructor.
   */
}

void PIDWithTwiddle::UpdateError(double cte) {
  /**
   * Update the PID error variables given cross track error.
   * It is the implementation of the Twiddle algorithm.
   * @param cte The current cross track error
   */
  if (!IsOptimizationFinished()) {
    std::vector<double> p = {Kp, Ki, Kd};

    if (TwiddleState::kStateInitial == optimization_state) {
      // Reset the running error and the first one.
      running_RMSE.Reset(rmse_count + 1);
      running_RMSE.Add(cte);

      // Compute next state
      optimization_state = TwiddleState::kStateComputeErrorAtInitial;
    } else if (TwiddleState::kStateComputeErrorAtInitial == optimization_state) {
      // Add the running error.
      running_RMSE.Add(cte);

      // Check if there are enough number of samples.
      if (running_RMSE.GetCount() > rmse_count) {
        // Initial RMS error.
        min_error = running_RMSE.getRMS();

        // Print Information
        std::cout << "Kp," << get_Kp();
        std::cout << ",Ki," << get_Ki();
        std::cout << ",Kd," << get_Kd();
        std::cout << ",error," << min_error;
        std::cout << ",min_error," << min_error;
        std::cout << std::endl;

        running_RMSE.Reset(rmse_count + 1);
        p[optimization_index] += dp[optimization_index];

        // Compute next state
        optimization_state = TwiddleState::kStateComputeErrorWhenIncrease;
      }
    } else if (TwiddleState::kStateComputeErrorWhenIncrease == optimization_state) {
      // Add the running error.
      running_RMSE.Add(cte);

      // Check if there are enough number of samples.
      if (running_RMSE.GetCount() > rmse_count) {
        // RMS error when increasing.
        double err = running_RMSE.getRMS();
        running_RMSE.Reset(rmse_count + 1);

        // Print Information
        std::cout << "Kp," << get_Kp();
        std::cout << ",Ki," << get_Ki();
        std::cout << ",Kd," << get_Kd();
        std::cout << ",err," << err;
        std::cout << ",min_error," << min_error;
        std::cout << std::endl;

        if (err < min_error) {
          // Use the increased parameter.
          min_error = err;

          // Update the step parameter trial.
          dp[optimization_index] *= 1.1;
          
          // Compute next state
          optimization_index = (optimization_index + 1) % 3;
          p[optimization_index] += dp[optimization_index];
          optimization_state = TwiddleState::kStateComputeErrorWhenIncrease;
        } else {
          // Try decreasing
          p[optimization_index] -= 2 * dp[optimization_index];
          
          // Compute next state
          optimization_state = TwiddleState::kStateComputeErrorWhenDecrease;
        }
      }
    } else if (TwiddleState::kStateComputeErrorWhenDecrease == optimization_state) {
      // Add the running error.
      running_RMSE.Add(cte);

      // Check if there are enough number of samples.
      if (running_RMSE.GetCount() > rmse_count) {
        double err = running_RMSE.getRMS();
        running_RMSE.Reset(rmse_count + 1);

        // Print Information
        std::cout << "Kp," << get_Kp();
        std::cout << ",Ki," << get_Ki();
        std::cout << ",Kd," << get_Kd();
        std::cout << ",err," << err;
        std::cout << ",min_error," << min_error;
        std::cout << std::endl;

        if (err < min_error) {
          // Use the decreased parameter.
          min_error = err;

          // Update the step parameter trial.
          dp[optimization_index] *= 1.1;
        } else {
          // Restore the parameter
          p[optimization_index] += dp[optimization_index];

          // Update the step parameter trial.
          dp[optimization_index] *= 0.9;
        }

        // Compute next state
        optimization_index = (optimization_index + 1) % 3;
        p[optimization_index] += dp[optimization_index];
        optimization_state = TwiddleState::kStateComputeErrorWhenIncrease;
      }
    }
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];
    PID::UpdateError(cte);
  } else {
    PID::UpdateError(cte);
  }
}

bool PIDWithTwiddle::startOptimization(int rmse_count, double toleranceRatio, std::vector<double> dp) {
  /**
   * Start the optimization using Twiddle algorithm.
   * @param rmse_count The number of samples used in computing root mean square eror
   * @param toleranceRatio The finishing tolerance ratio related to the initial step parameters
   * @param dp The twiddle step parameters
   * @output if the start of the optimization is successful
   */
  if (!IsOptimizationFinished()) {
    return false;
  }

  this->rmse_count = rmse_count;
  this->dp = dp;
  this->dpFinished = {dp[0] * toleranceRatio, dp[1] * toleranceRatio, dp[2] * toleranceRatio};
  optimization_index = 0;
  optimization_state = TwiddleState::kStateComputeErrorAtInitial;
  running_RMSE.Reset(rmse_count + 1);

  return true;
}

bool PIDWithTwiddle::IsOptimizationFinished() {
  /**
   * Check if the the optimization is finished.
   * @output if the the optimization is finished
   */
  bool finished = ((dp[0] < dpFinished[0]) && (dp[1] < dpFinished[1]) && (dp[2] < dpFinished[2]));
  return finished;
}