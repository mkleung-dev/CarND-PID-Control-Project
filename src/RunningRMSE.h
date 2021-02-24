#ifndef RUNNING_RMSE_H
#define RUNNING_RMSE_H

#include <deque>

class RunningRMSE {
 public:
  /**
   * Constructor
   */
  RunningRMSE(int max_count = 100);
  /**
   * Destructor.
   */
  virtual ~RunningRMSE();

  void AddError(double err);
  double getRMSE();
  void Reset(int max_count);
  int GetCount();

 protected:
  std::deque<double> sqr_err;
  int max_count;
};

#endif  // RUNNING_RMSE_H