#include "Timer.hpp"



#ifdef _WIN32
#include <Windows.h>
#elif unix
#endif

#include <iostream>


Timer::Timer(const double& rate_){

	// Set the Timer rate
	this->setRate(rate_);

}

Timer::~Timer(){}

double Timer::elapsedTime(const double& tic, const double& toc){

	double dt;

#ifdef _WIN32
	LARGE_INTEGER rate;
	QueryPerformanceFrequency(&rate);
	dt = (toc - tic)/(double)rate.QuadPart;
#elif unix

#endif

	return dt;
}

double Timer::getCurTime(){

	double time;

#ifdef _WIN32
	LARGE_INTEGER time_i;
	QueryPerformanceCounter(&time_i);
	time = (double)time_i.QuadPart;
#elif unix

#endif


	return time;

}

void Timer::setRate(const double& r){

	this->rate = r;

}

void Timer::timeSleep(const double& sec){

#ifdef _WIN32
	Sleep((DWORD)1000 * static_cast<DWORD>(sec)); // Sleep(ms)
#elif unix

#endif

}

