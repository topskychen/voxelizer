/*
 * ThreadPool.cpp
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#include "threadPool.h"

ThreadPool::ThreadPool(size_t nThreads) {
	Restart(nThreads);
}

void ThreadPool::Stop(bool wait) {
	if (wait) {
	  _work.reset(); // if not delete this, io_service::run will never exit
	  _threadPool.join_all();
	  _service.stop();
	} else {
	  _service.stop();
	  _threadPool.join_all();
	  _work.reset();
	}
}

void ThreadPool::Restart(size_t nThreads) {
	_work.reset(new boost::asio::io_service::work(_service));
		for (std::size_t i = 0; i < nThreads; ++i)
			_threadPool.create_thread(boost::bind(&boost::asio::io_service::run, &_service));
}

ThreadPool::~ThreadPool() {
	_threadPool.join_all();
}

