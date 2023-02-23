#ifndef MOVINGAVERAGEFILTER_H_
#define MOVINGAVERAGEFILTER_H_
#include <ArduinoSTL.h>
#include <deque>
#include <vector>

class MovingAverageFilter final {
public:
  // Construct a new Moving Average Filter object
  MovingAverageFilter(int sample_size);
  // Add new data to filter
  void addSample(int new_sample);
  // Get filtered data
  float getWeightedMovingAverage();
  // Get filtered data
  float getMovingAverage();

private:
  const uint8_t kSampleSize_;
  std::deque<int> samples_;
  std::vector<uint8_t> weight_;
};

#endif  // MOVING_AVERAGE_FILTER_H_