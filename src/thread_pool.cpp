/*
 * ThreadPool.cpp
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#include "thread_pool.h"

namespace voxelizer {

ThreadPool::ThreadPool(size_t n_threads) { Restart(n_threads); }

void ThreadPool::Stop(bool wait) {
  if (wait) {
    work_.reset();  // if not delete this, io_service::run will never exit
    thread_pool_.join_all();
    service_.stop();
  } else {
    service_.stop();
    thread_pool_.join_all();
    work_.reset();
  }
}

void ThreadPool::Restart(size_t nThreads) {
  work_.reset(new boost::asio::io_service::work(service_));
  for (std::size_t i = 0; i < nThreads; ++i)
    thread_pool_.create_thread(
        boost::bind(&boost::asio::io_service::run, &service_));
}

ThreadPool::~ThreadPool() { thread_pool_.join_all(); }

}  // namespace voxelizer
