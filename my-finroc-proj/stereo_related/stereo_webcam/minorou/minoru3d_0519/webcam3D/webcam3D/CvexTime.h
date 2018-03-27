#include <cv.h>


enum
{
	CVEX_TIME_NS=0,
	CVEX_TIME_MS,
	CVEX_TIME_SEC,
	CVEX_TIME_MIN,
	CVEX_TIME_HOUR
};
class CV_EXPORTS CvexTime
{
	int64 pre;
	char mes[256];

	int timeMode;

	double cTime;
	bool _isShow;

	void start();
	void stop();
public:

	void setMode(int mode);
	void setMessage(char* src);
	void restart();
	double getTime();
	void show();
	
	
	CvexTime(int mode=CVEX_TIME_MS, char*message="time ",bool isShow=true);
	~CvexTime();
	
};