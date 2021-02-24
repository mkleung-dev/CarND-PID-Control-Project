#include "RunningRMSE.h"

#include <math.h>

RunningRMSE::RunningRMSE(int max_count) {
  Reset(max_count);
}

RunningRMSE::~RunningRMSE() {
}

void RunningRMSE::Reset(int max_count) {
  this->max_count = max_count;
  sqr_err.clear();
}

void RunningRMSE::AddError(double err) {
  sqr_err.push_back(err * err);
  while (sqr_err.size() > max_count) {
    sqr_err.pop_front();
  }
}

double RunningRMSE::getRMSE() {
  double err = 0;
  for (int i = 0; i < sqr_err.size(); i++) {
    err += sqr_err[i];
  }
  return err / sqr_err.size();
}

int RunningRMSE::GetCount() {
  return sqr_err.size();
}