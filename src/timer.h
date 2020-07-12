/*
 * Timer.h
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "absl/time/time.h"

namespace voxelizer {

class Timer {
  absl::Time start_, end_;

 public:
  Timer();
  virtual ~Timer();
  void Restart();
  void Stop();
  int64_t TimeInS();
  int64_t TimeInMs();
  void PrintTimeInS();
  void PrintTimeInMs();
};

}  // namespace voxelizer

#endif /* TIMER_H_ */
