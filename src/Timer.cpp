/*
 * Timer.cpp
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#include "timer.h"

Timer::Timer() {
}


void Timer::Restart() {
	gettimeofday(&_time, NULL);
	_start = _time.tv_sec+(_time.tv_usec/1000000.0);
}

void Timer::Stop() {
	gettimeofday(&_time, NULL);
	_end = _time.tv_sec+(_time.tv_usec/1000000.0);
}

double Timer::TimeInS() {
	double res = (_end - _start);
	return res;
}


Timer::~Timer() {
}

void Timer::PrintTimeInS() {
	std::cout << "consumes " << TimeInS() << " s." << std:: endl;
}
