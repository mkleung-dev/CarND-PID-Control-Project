#ifndef RUNNING_RMSE_H
#define RUNNING_RMSE_H

class RunningRMSE {
 public:
  /**
   * Constructor
   */
  RunningRMSE();
  /**
   * Destructor.
   */
  virtual ~RunningRMSE();

  void AddError(double err);
  int GetCount();
  double getRMSE();
  void Reset();

 protected:
  double acc_square_err;
  int count;
};

#endif  // RUNNING_RMSE_H