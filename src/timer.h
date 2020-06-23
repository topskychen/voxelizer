/*
 * Timer.h
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#ifndef TIMER_H_
#define TIMER_H_
#include <sys/time.h>

#include <ctime>
#include <iostream>

namespace voxelizer {

class Timer {
  struct timeval time_;
  double start_, end_;

 public:
  Timer();
  virtual ~Timer();
  void Restart();
  void Stop();
  double TimeInS();
  void PrintTimeInS();
};

}  // namespace voxelizer

#endif /* TIMER_H_ */
