/*
 * ThreadPool.h
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#ifndef THREADPOOL_H_
#define THREADPOOL_H_

#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

namespace voxelizer {

class ThreadPool {
  boost::shared_ptr<boost::asio::io_service::work> work_;
  boost::asio::io_service service_;
  boost::thread_group thread_pool_;

 public:
  void Stop(bool wait = true);
  ThreadPool(size_t n_threads = 1);
  void Restart(size_t n_threads = 1);
  template <typename T>
  void Run(T func) {
    service_.post(func);
  }
  virtual ~ThreadPool();
};

}  // namespace voxelizer

#endif /* THREADPOOL_H_ */
