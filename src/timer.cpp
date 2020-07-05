/*
 * Timer.cpp
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#include "timer.h"

namespace voxelizer {

Timer::Timer() {}

void Timer::Restart() {
  gettimeofday(&time_, NULL);
  start_ = time_.tv_sec + (time_.tv_usec / 1000000.0);
}

void Timer::Stop() {
  gettimeofday(&time_, NULL);
  end_ = time_.tv_sec + (time_.tv_usec / 1000000.0);
}

double Timer::TimeInS() {
  double res = (end_ - start_);
  return res;
}

Timer::~Timer() {}

void Timer::PrintTimeInS() {
  std::cout << "consumes " << TimeInS() << " s." << std::endl;
}

}  // namespace voxelizer