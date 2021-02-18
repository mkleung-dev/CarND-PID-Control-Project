#include "RunningRMSE.h"

#include <math.h>

RunningRMSE::RunningRMSE() {
  Reset();
}

RunningRMSE::~RunningRMSE() {
}

void RunningRMSE::Reset() {
  count = 0;
  acc_square_err = 0;
}

void RunningRMSE::AddError(double err) {
  acc_square_err += err * err;
  count++;
}

int RunningRMSE::GetCount() {
  return count;
}

double RunningRMSE::getRMSE() {
  return sqrt(acc_square_err) / count;
}