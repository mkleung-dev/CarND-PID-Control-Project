#include "RunningData.h"

#include <math.h>

RunningData::RunningData(int max_count) {
  /**
   * Constructor
   */
  Reset(max_count);
}

RunningData::~RunningData() {
  /**
   * Destructor.
   */
  sqr_values.clear();
  values.clear();
}

void RunningData::Reset(int max_count) {
  /**
   * Reset the running data.
   * @param max_count The maximum number of samples in the running data
   */
  this->max_count = max_count;
  sqr_values.clear();
  values.clear();
}

void RunningData::Add(double value) {
  /**
   * Add single running data.
   * @param value The single running data
   */
  sqr_values.push_back(value * value);
  while (sqr_values.size() > max_count) {
    sqr_values.pop_front();
  }
  values.push_back(value);
  while (values.size() > max_count) {
    values.pop_front();
  }
}

double RunningData::getRMS() {
  /**
   * Get the root mean square of the running data.
   * @output the root mean square of the running data
   */
  double err = 0;
  for (int i = 0; i < sqr_values.size(); i++) {
    err += sqr_values[i];
  }
  return err / sqr_values.size();
}

double RunningData::getMean() {
  /**
   * Get the mean of the running data.
   * @output the mean of the running data
   */
  double err = 0;
  for (int i = 0; i < values.size(); i++) {
    err += values[i];
  }
  return err / values.size();
}

int RunningData::GetCount() {
  /**
   * Get the number of samples in the running data.
   * @output the number of samples in the running data
   */
  return sqr_values.size();
}