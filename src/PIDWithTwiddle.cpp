#include "PIDWithTwiddle.h"

#include <iostream>
#include <vector>


PIDWithTwiddle::PIDWithTwiddle() : PID() {
  min_error = 99999999;
  optimization_index = 0;
  optimization_state = TwiddleState::kState1;
  dpFinished = {100, 100, 100};
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
    // std::cout << "dp0," << dp[0] << ",dp1," << dp[1] << ",dp2," << dp[2] << std::endl;

    if (TwiddleState::kState0 == optimization_state) {
      running_RMSE.Reset(rmse_count + 1);

      optimization_state = TwiddleState::kState1;
    } else if (TwiddleState::kState1 == optimization_state) {
      running_RMSE.AddError(cte);

      if (running_RMSE.GetCount() > rmse_count) {
        min_error = running_RMSE.getRMSE();
        running_RMSE.Reset(rmse_count + 1);
        p[optimization_index] += dp[optimization_index];

        optimization_state = TwiddleState::kState2;
      }
    } else if (TwiddleState::kState2 == optimization_state) {
      running_RMSE.AddError(cte);

      if (running_RMSE.GetCount() > rmse_count) {
        double err = running_RMSE.getRMSE();
        //double max_error = running_RMSE.getMaxErr();
        running_RMSE.Reset(rmse_count + 1);

        if (err < min_error/* && max_error < 10.0*/) {
          //Update Current
          min_error = err;
          dp[optimization_index] *= 1.1;
          //Prepare for the Next
          optimization_index = (optimization_index + 1) % 3;
          p[optimization_index] += dp[optimization_index];
          optimization_state = TwiddleState::kState2;
        } else {
          //Update Current
          p[optimization_index] -= 2 * dp[optimization_index];
          //Prepare for the Next
          optimization_state = TwiddleState::kState3;
        }
      }
    } else if (TwiddleState::kState3 == optimization_state) {
      running_RMSE.AddError(cte);

      if (running_RMSE.GetCount() > rmse_count) {
        double err = running_RMSE.getRMSE();
        //double max_error = running_RMSE.getMaxErr();
        running_RMSE.Reset(rmse_count + 1);

        if (err < min_error/* && max_error < 10.0*/) {
          //Update Current
          min_error = err;
          dp[optimization_index] *= 1.1;
        } else {
          //Update Current
          p[optimization_index] += dp[optimization_index];
          dp[optimization_index] *= 0.9;
        }
        //Prepare for the Next
        optimization_index = (optimization_index + 1) % 3;
        p[optimization_index] += dp[optimization_index];
        optimization_state = TwiddleState::kState2;
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

bool PIDWithTwiddle::startOptimization(int rmse_count, double toleranceRatio, std::vector<double> dp) {
  if (!IsOptimizationFinished()) {
    return false;
  }

  // std::cout << "dp0," << dp[0] << ",dp1," << dp[1] << ",dp2," << dp[2] << std::endl;
  this->rmse_count = rmse_count;
  this->dp = dp;
  this->dpFinished = {dp[0] * toleranceRatio, dp[1] * toleranceRatio, dp[2] * toleranceRatio};
  optimization_index = 0;
  optimization_state = TwiddleState::kState1;
  running_RMSE.Reset(rmse_count + 1);

  return true;
}

bool PIDWithTwiddle::IsOptimizationFinished() {
  bool finished = ((dp[0] < dpFinished[0]) && (dp[1] < dpFinished[1]) && (dp[2] < dpFinished[2]));
  return finished;
}

TwiddleState PIDWithTwiddle::getState() {
  return optimization_state;
}

int PIDWithTwiddle::getOptimizationIndex() {
  return optimization_index;
}

int PIDWithTwiddle::getRMSECount() {
  return running_RMSE.GetCount();
}

double PIDWithTwiddle::getDp(int index) {
  return dp[index];
}

double PIDWithTwiddle::getDpFinished(int index) {
  return dpFinished[index];
}