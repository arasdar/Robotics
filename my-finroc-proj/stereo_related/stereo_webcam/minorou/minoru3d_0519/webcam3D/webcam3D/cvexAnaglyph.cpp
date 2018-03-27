#include "mcv.h"

void cvexMakeStereoImageAnaglyph(IplImage* lim,IplImage* rim,IplImage* dest, int d)
{
	IplImage* g1=cvCreateImage(cvGetSize(lim),8,1);
	IplImage* g2=cvCreateImage(cvGetSize(lim),8,1);
	IplImage* g3=cvCreateImage(cvGetSize(lim),8,1);
	IplImage* swap=cvCreateImage(cvGetSize(lim),8,1);


	//	cvSplit(rim,swap,NULL,NULL,NULL);
	cvCvtColor(lim,swap,CV_BGR2GRAY);
	cvexWarpShift(swap,g1,-d,0);

	//cvSplit(lim,NULL,NULL,swap,NULL);
	cvCvtColor(rim,swap,CV_BGR2GRAY);
	cvexWarpShift(swap,g2,d,0);


	cvMerge(g2,g2,g1,NULL,dest);

	cvReleaseImage(&g1);
	cvReleaseImage(&g2);
	cvReleaseImage(&g3);
	cvReleaseImage(&swap);
}