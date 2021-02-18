#include "PIDWithTwiddle.h"

#include <iostream>
#include <vector>


PIDWithTwiddle::PIDWithTwiddle() : PID() {
  tolerance = 99999999;
  min_error = 99999999;
  optimization_index = 0;
  optimization_state = TwiddleState::kState1;
  dp = {0, 0, 0};
}

PIDWithTwiddle::~PIDWithTwiddle() {
  
}

void PIDWithTwiddle::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  if (!IsOptimizationFinished()) {
    std::vector<double> p = {Kp, Ki, Kd};
    std::cout << "dp0," << dp[0] << ",dp1," << dp[1] << ",dp2," << dp[2] << std::endl;


    if (TwiddleState::kState1 == optimization_state) {
      p[optimization_index] += dp[optimization_index];

      optimization_state = TwiddleState::kState2;
      running_RMSE.Reset();
    } else if (TwiddleState::kState2 == optimization_state) {
      running_RMSE.AddError(cte);

      if (running_RMSE.GetCount() > rmse_count) {
        running_RMSE.Reset();
        if (cte < min_error) {
          min_error = cte;
          dp[optimization_index] *= 1.1;

          optimization_index = (optimization_index + 1) % 3;
          optimization_state = TwiddleState::kState1;
        } else {
          p[optimization_index] -= 2 * dp[optimization_index];

          optimization_state = TwiddleState::kState3;
          running_RMSE.Reset();
        }
      }
    } else if (TwiddleState::kState3 == optimization_state) {
      running_RMSE.AddError(cte);

      if (running_RMSE.GetCount() > rmse_count) {
        running_RMSE.Reset();
        if (cte < min_error) {
          min_error = cte;
          dp[optimization_index] *= 1.1;

          optimization_index = (optimization_index + 1) % 3;
          optimization_state = TwiddleState::kState1;
        } else {
          p[optimization_index] += dp[optimization_index];
          dp[optimization_index] *= 0.9;

          optimization_index = (optimization_index + 1) % 3;
          optimization_state = TwiddleState::kState1;
        }
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

double PIDWithTwiddle::TotalError() {
  /**
   * Calculate and return the total error
   */
  return Kp * p_error + Ki * i_error + Kd * d_error;
}

bool PIDWithTwiddle::startOptimization(int rmse_count, double tolerance, std::vector<double> dp) {
  if (!IsOptimizationFinished()) {
    return false;
  }

  std::cout << "dp0," << dp[0] << ",dp1," << dp[1] << ",dp2," << dp[2] << std::endl;
  this->rmse_count = rmse_count;
  this->tolerance = tolerance;
  this->dp = dp;
  optimization_index = 0;
  optimization_state = TwiddleState::kState1;
  running_RMSE.Reset();

  return true;
}

bool PIDWithTwiddle::IsOptimizationFinished() {
  bool finished = ((dp[0] + dp[1] + dp[2]) < tolerance);
  std::cout << "Finished:" << (finished ? "YES" : "NO") << std::endl;
  return finished;
}