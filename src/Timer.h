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

	struct timeval _time;
	double _start, _end;

public:
	Timer();
	virtual ~Timer();
	void Restart();
	void Stop();
	double TimeInS();
	void PrintTimeInS();
};

#endif /* TIMER_H_ */
