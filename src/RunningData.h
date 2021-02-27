#ifndef RUNNING_DATA_H
#define RUNNING_DATA_H

#include <deque>

/**
 * The Running Data class.
 */
class RunningData {
 public:
  /**
   * Constructor
   */
  RunningData(int max_count = 100);
  /**
   * Destructor.
   */
  virtual ~RunningData();

  /**
   * Add single running data.
   * @param value The single running data
   */
  void Add(double value);

  /**
   * Get the root mean square of the running data.
   * @output the root mean square of the running data
   */
  double getRMS();

  /**
   * Get the mean of the running data.
   * @output the mean of the running data
   */
  double getMean();

  /**
   * Reset the running data.
   * @param max_count The maximum number of samples in the running data
   */
  void Reset(int max_count);

  /**
   * Get the number of samples in the running data.
   * @output the number of samples in the running data
   */
  int GetCount();

 protected:
  /**
   * Running data
   */
  std::deque<double> values;
  std::deque<double> sqr_values;
  /**
   * The maximum number of samples in the running data
   */
  int max_count;
};

#endif  // RUNNING_DATA_H