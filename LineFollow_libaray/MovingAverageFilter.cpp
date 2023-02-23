#include "MovingAverageFilter.h"

MovingAverageFilter::MovingAverageFilter(int sample_size)
    : kSampleSize_(sample_size) {
  weight_.reserve(kSampleSize_);
  for (uint8_t i = 1; i <= kSampleSize_; ++i) {
    weight_.push_back(i);
  }
}

void MovingAverageFilter::addSample(int new_sample) {
  samples_.push_back(new_sample);
  if (samples_.size() > kSampleSize_) {
    samples_.pop_front();
  }
}

float MovingAverageFilter::getMovingAverage() {
  int sum = 0, sample_size = samples_.size();
  for (uint8_t i = 0; i < sample_size; ++i) {
    sum += samples_[i];
  }
  return (float)sum / sample_size;
}

float MovingAverageFilter::getWeightedMovingAverage() {
  int sum = 0, weight_sum = 0;
  for (uint8_t i = 0; i < samples_.size(); ++i) {
    sum += samples_[i] * weight_[i];
    weight_sum += weight_[i];
  }
  if (weight_sum == 0) {
   throw std::runtime_error("Weight sum is zero");
  }
  return (float)sum / weight_sum;
}
