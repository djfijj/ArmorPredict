#pragma once
#include <windows.h>
class CntTime
{
public:
	CntTime(void);
	~CntTime(void);//Îö¹¹º¯Êý

private:
	LARGE_INTEGER start_time;

	LARGE_INTEGER end_time;

	LARGE_INTEGER CPU_rate;

public:
	double interval;

public:
	void start();
	void end();
};



CntTime::CntTime(void)
{
	QueryPerformanceFrequency(&CPU_rate);
}

CntTime::~CntTime(void)
{
}

void CntTime::start()
{
	QueryPerformanceCounter(&start_time);
}

void CntTime::end()
{
	QueryPerformanceCounter(&end_time);

	interval = ((double)end_time.QuadPart - (double)start_time.QuadPart) / (double)CPU_rate.QuadPart;

}