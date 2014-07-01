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
#include <boost/thread/thread.hpp>
#include <boost/function.hpp>

class ThreadPool {
	boost::shared_ptr<boost::asio::io_service::work> _work;
	boost::asio::io_service _service;
	boost::thread_group _threadPool;

public:
	void Stop(bool wait=true);
	ThreadPool(size_t nThreads = 1);
	void Restart(size_t nThreads = 1);
	template<typename T>
	void Run(T func) {
		_service.post(func);
	}
	virtual ~ThreadPool();
};

#endif /* THREADPOOL_H_ */
