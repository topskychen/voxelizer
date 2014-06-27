/*
 * Timer.cpp
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#include "Timer.h"

Timer::Timer() {
}


void Timer::restart() {
	gettimeofday(&time, NULL);
	start = time.tv_sec+(time.tv_usec/1000000.0);
}

void Timer::stop() {
	gettimeofday(&time, NULL);
	end = time.tv_sec+(time.tv_usec/1000000.0);
}

double Timer::timeInS() {
	double res = (end - start);
	return res;
}


Timer::~Timer() {
}

void Timer::printTimeInS() {
	std::cout << "consumes " << timeInS() << " s." << std:: endl;
}
