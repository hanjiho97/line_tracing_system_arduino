#include "MovingAverageFilter.h"

MovingAverageFilter::MovingAverageFilter(uint8_t sample_size)
    : kSampleSize_(sample_size) 
{
  weight_.reserve(kSampleSize_);
  for (uint8_t i = 1U; i <= kSampleSize_; ++i) 
  {
    weight_.push_back(i);
  }
}

void MovingAverageFilter::add_sample(uint16_t new_sample) 
{
  samples_.push_back(new_sample);
  if (samples_.size() > kSampleSize_) 
  {
    samples_.pop_front();
  }
}

float MovingAverageFilter::get_moving_average() 
{
  uint16_t sum = 0;
  uint8_t sample_size = samples_.size();
  for (uint8_t i = 0U; i < sample_size; ++i) 
  {
    sum += samples_[i];
  }
  return static_cast<float>(sum) / static_cast<float>(sample_size);
}

float MovingAverageFilter::get_weighted_moving_average() 
{
  uint32_t sum = 0;
  uint16_t weight_sum = 0;
  for (uint8_t i = 0U; i < samples_.size(); ++i) 
  {
    sum += samples_[i] * weight_[i];
    weight_sum += weight_[i];
  }
  return static_cast<float>(sum) / static_cast<float>(weight_sum);
}
