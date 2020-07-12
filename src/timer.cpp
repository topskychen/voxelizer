/*
 * Timer.cpp
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#include <iostream>

#include "timer.h"
#include "absl/time/clock.h"

namespace voxelizer {

Timer::Timer() {}

void Timer::Restart() {
  start_ = absl::Now();
}

void Timer::Stop() {
  end_ = absl::Now();
}

int64_t Timer::TimeInS() {
  absl::Duration dur = end_ - start_;
  return dur / absl::Seconds(1);
}

int64_t Timer::TimeInMs() {
  absl::Duration dur = end_ - start_;
  return dur / absl::Milliseconds(1);
}

Timer::~Timer() {}

void Timer::PrintTimeInS() {
  std::cout << "consumes " << TimeInS() << " s." << std::endl;
}

void Timer::PrintTimeInMs() {
  std::cout << "consumes " << TimeInMs() << " ms." << std::endl;
}

}  // namespace voxelizer