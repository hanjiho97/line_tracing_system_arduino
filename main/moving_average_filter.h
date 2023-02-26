#ifndef _MOVING_AVERAGE_FILTER_H_
#define _MOVING_AVERAGE_FILTER_H_

#include <ArduinoSTL.h>
#include <vector>
#include <deque>

#include "common_params.h"


class MovingAverageFilter final
{
public:
  // Construct a new Moving Average Filter object
  MovingAverageFilter(uint8_t sample_size);
  // Add new data to filter
  void add_sample(uint16_t new_sample);
  // Get filtered data
  float get_weighted_moving_average();
  // Get filtered data
  float get_moving_average();

private:
  const uint8_t sample_size_;
  std::deque<uint16_t> samples_;
  std::vector<uint8_t> weight_;
};

#endif // MOVING_AVERAGE_FILTER_H_
