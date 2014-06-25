/*
 * Timer.h
 *
 *  Created on: 22 Jun, 2014
 *      Author: chenqian
 */

#ifndef TIMER_H_
#define TIMER_H_
#include <ctime>
#include <iostream>
#include <sys/time.h>

class Timer {

	struct timeval time;
	double start, end;

public:
	Timer();
	virtual ~Timer();
	void restart();
	void stop();
	double timeInS();
	void printTimeInS();
};

#endif /* TIMER_H_ */
