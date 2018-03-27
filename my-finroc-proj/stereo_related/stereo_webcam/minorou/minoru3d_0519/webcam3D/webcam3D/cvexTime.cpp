
#include "CvexTime.h"
#include <iostream>
using namespace std;

void CvexTime::start()
{
	pre = cvGetTickCount();
}


void CvexTime::restart()
{
	start();
}

void CvexTime:: show()
{
	switch(timeMode)
	{
	case CVEX_TIME_NS:
		cout<< mes<< ": "<<cTime<<" ns"<<endl;
		break;
	case CVEX_TIME_SEC:
		cout<< mes<< ": "<<cTime<<" sec"<<endl;
		break;
	case CVEX_TIME_MIN:
		cout<< mes<< ": "<<cTime<<" minute"<<endl;
		break;
	case CVEX_TIME_HOUR:
		cout<< mes<< ": "<<cTime<<" hour"<<endl;
		break;

	case CVEX_TIME_MS:
	default:
		cout<<mes<< ": "<<cTime<<" ms"<<endl;
		break;
	}
}

void CvexTime:: stop()
{
	switch(timeMode)
	{
	case CVEX_TIME_NS:
		cTime = (cvGetTickCount()-pre)/(cvGetTickFrequency());
		break;
	case CVEX_TIME_SEC:
		cTime = (cvGetTickCount()-pre)/(cvGetTickFrequency())/1000000.0;
		break;
	case CVEX_TIME_MIN:
		cTime = (cvGetTickCount()-pre)/(cvGetTickFrequency())/(1000000.0*60);
		break;
	case CVEX_TIME_HOUR:
		cTime = (cvGetTickCount()-pre)/(cvGetTickFrequency())/(1000000.0*60*60);
		break;

	case CVEX_TIME_MS:
	default:
		cTime = (cvGetTickCount()-pre)/(cvGetTickFrequency())/1000.0;
		break;
	}
}
double CvexTime:: getTime()
{
	stop();
	return cTime;
}

void CvexTime:: setMessage(char* src)
{
	strcpy(mes,src);
}
void CvexTime:: setMode(int mode)
{

	timeMode = mode;
}

CvexTime::CvexTime(int mode, char* message,bool isShow)
{
	_isShow = isShow;
	timeMode = mode;

	setMessage(message);
	start();
}
CvexTime::~CvexTime()
{
	stop();
	if(_isShow)	show();
}