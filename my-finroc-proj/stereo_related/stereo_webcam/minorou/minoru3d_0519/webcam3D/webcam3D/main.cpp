#include <iostream>
using namespace std;

#include <direct.h>

#include <omp.h>

#include "mcv.h"
#include "CvexTime.h"
#include "CvexStereoCameraCalibration.h"

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define CHESS_SIZE cvSize(7,4)


enum
{
	MINORU_VIEW_LEFT=0,
	MINORU_VIEW_RIGHT,
	MINORU_VIEW_ANAGLYPH,
	MINORU_VIEW_INTERLACE,
	MINORU_VIEW_SIDEBYSIDE,

	MINORU_VIEW_DEPTHMAP,//max value for trackbar
};
void editHomographyGUI(IplImage* leftim, IplImage* rightim,CvMat* hl,CvMat* hr)
{
	CvMat* H_L = cvCloneMat(hl);
	CvMat* H_R = cvCloneMat(hr);
	cvNamedWindow ("Homography", CV_WINDOW_AUTOSIZE);
	int v_max=leftim->height;
	int h_max=leftim->width;
	int v=v_max/2;
	int h = h_max/2;
	cvCreateTrackbar("v","Homography",&v,v_max,NULL);
	cvCreateTrackbar("h","Homography",&h,h_max,NULL);

	IplImage* render = cvCloneImage(leftim);
	int key = 0;
	while (key !='\x1b') 
	{
		
		cvWarpPerspective(leftim,render,hl);

		hl->data.db[2]=H_L->data.db[2]+h-h_max/2;
		hr->data.db[2]=H_R->data.db[2]+h-h_max/2;

		hl->data.db[5]=H_L->data.db[5]+v-v_max/2;
		hr->data.db[5]=H_R->data.db[5]+v-v_max/2;

		if(key==CVEX_KEY_ARROW_UP)
		{
			v--;
			cvSetTrackbarPos("v","Homography",v);
		}
		if(key==CVEX_KEY_ARROW_DOWN)
		{
			v++;
			cvSetTrackbarPos("v","Homography",v);
		}
		if(key==CVEX_KEY_ARROW_LEFT)
		{
			h--;
			cvSetTrackbarPos("h","Homography",h);
		}
		if(key==CVEX_KEY_ARROW_RIGHT)
		{
			h++;
			cvSetTrackbarPos("h","Homography",h);
		}

		cvShowImage ("Homography", render);
		key = cvWaitKey (1);
	}
	cvDestroyWindow("Homography");
	cvReleaseImage(&render);
	cvReleaseMat(&H_L);
	cvReleaseMat(&H_R);

}
void getShiftRectificationParameterGUI(IplImage* leftim, IplImage* rightim,int *_v)
{
	int v_max = leftim->height/5;
	int h_max = leftim->width;
	cvNamedWindow ("shift rectification", CV_WINDOW_AUTOSIZE);
	int v=*_v+v_max/2;

	cvCreateTrackbar("v","shift rectification",&v,v_max,NULL);
	int h = h_max/2;
	cvCreateTrackbar("h","shift rectification",&h,h_max,NULL);

	IplImage* render = cvCloneImage(leftim);
	int key = 0;
	while (key !='\x1b') 
	{
		cvexWarpShift(rightim,render,h-h_max/2,v-v_max/2);
		cvAddWeighted(leftim,0.5,render,0.5,0,render);

		cvShowImage ("shift rectification", render);

		key = cvWaitKey (1);
	}

	*_v=v-v_max/2;
	cvDestroyWindow("shift rectification");
	cvReleaseImage(&render);
}

void depth_estimation(IplImage* leftim, IplImage* rightim, IplImage* dest, int maxDisparity, 
					  int occ_penalty=15, int match_reward=3, int good_match=6,int middle_match=8, int bad_match=15)
{
	IplImage* depth = cvCreateImage(cvGetSize(leftim),8,1);
	IplImage* l = cvCreateImage(cvGetSize(leftim),8,1);
	IplImage* r = cvCreateImage(cvGetSize(leftim),8,1);

	cvCvtColor(leftim,l,CV_BGR2GRAY);
	cvCvtColor(rightim,r,CV_BGR2GRAY);

	cvFindStereoCorrespondence( l, r, CV_DISPARITY_BIRCHFIELD, depth, maxDisparity, occ_penalty, match_reward,good_match, middle_match, bad_match );

	cvCvtColor(depth,dest,CV_GRAY2BGR);
	cvScale(dest,dest,255/maxDisparity);
	cvReleaseImage(&depth);
	cvReleaseImage(&l);
	cvReleaseImage(&r);
}

void cvexMakeStereoImageSidebySide(IplImage* left, IplImage* right, IplImage* dest, int shift, int mode = CVEX_CONNECT_HORIZON)
{
	IplImage* swap = cvCreateImage(cvGetSize(left),8,left->nChannels);
	IplImage* temp = cvCreateImage(cvGetSize(left),8,left->nChannels);
	cvexWarpShift(left,swap,-shift,0);
	cvexWarpShift(right,temp,+shift,0);

	IplImage* connect = cvexConnect(swap,temp,mode);

	cvResize(connect,dest);


	cvReleaseImage(&connect);
	cvReleaseImage(&swap);
	cvReleaseImage(&temp);

}
void cvexMakeStereoImageInterlace(IplImage* left, IplImage* right, IplImage* dest, int shift)
{
	cvexWarpShift(left,dest,-shift,0);
	IplImage* swap = cvCreateImage(cvGetSize(right),8,3);
	cvexWarpShift(right,swap,+shift,0);
#pragma omp parallel for
	for(int j=0;j<left->height;j++)
	{
		if(j%2==1)
		{
			for(int i=0;i<left->width;i++)
			{
				dest->imageData[j*left->widthStep+i*3+0]=swap->imageData[j*left->widthStep+i*3+0];
				dest->imageData[j*left->widthStep+i*3+1]=swap->imageData[j*left->widthStep+i*3+1];
				dest->imageData[j*left->widthStep+i*3+2]=swap->imageData[j*left->widthStep+i*3+2];
			}
		}
	}
	cvReleaseImage(&swap);
}


void demo()
{
	_mkdir("calibration_image");
	_mkdir("capture");
	CvCapture *leftcam = 0;
	CvCapture *rightcam = 0;
	IplImage *leftim = 0;
	IplImage *rightim = 0;

	CvMat* hright = cvCreateMat(3,3,CV_64F);
	CvMat* hleft = cvCreateMat(3,3,CV_64F);
	cvSetIdentity(hleft);
	cvSetIdentity(hright);

	double w = IMAGE_WIDTH, h = IMAGE_HEIGHT;

	leftcam = cvCreateCameraCapture (0);
	rightcam = cvCreateCameraCapture (1);

	//camera setup(width and height)
	cvSetCaptureProperty (leftcam, CV_CAP_PROP_FRAME_WIDTH, w);
	cvSetCaptureProperty (leftcam, CV_CAP_PROP_FRAME_HEIGHT, h);
	cvSetCaptureProperty (rightcam, CV_CAP_PROP_FRAME_WIDTH, w);
	cvSetCaptureProperty (rightcam, CV_CAP_PROP_FRAME_HEIGHT, h);

	leftim = cvQueryFrame (leftcam);
	IplImage* render = cvCloneImage(leftim);

	cvNamedWindow ("3D Webcam", CV_WINDOW_AUTOSIZE);
	int mode =0;
	cvCreateTrackbar("switch","3D Webcam",&mode,MINORU_VIEW_DEPTHMAP,NULL);

	int dis = render->width/2;
	cvCreateTrackbar("parallax","3D Webcam",&dis,render->width,NULL);

	int v_shift=0;
	int key=0;

	bool isRectification = false;

	CvexStereoCameraCalibration calib(cvGetSize(render),CHESS_SIZE,30);

	bool isLRFlip = false;



	while (key !='\x1b') //ECS
	{
		//CvexTime t(CVEX_TIME_MS,"capture");
		leftim = cvQueryFrame (leftcam);
		rightim = cvQueryFrame (rightcam);


		if(key=='r')getShiftRectificationParameterGUI(leftim,rightim,&v_shift);
		if(key=='t')
		{
			cvWarpPerspective(leftim,render,hleft);
			cvCopy(render,leftim);
			cvWarpPerspective(rightim,render,hright);
			cvCopy(render,rightim);
			getShiftRectificationParameterGUI(leftim,rightim,&v_shift);
		}
		if(key =='c')
		{
			bool is = calib.findChess(leftim,rightim);

			if(is)
			{
				cvexSaveImage(leftim,"calibration_image\\left%03d.bmp",calib.getImageCount());
				cvexSaveImage(rightim,"calibration_image\\right%03d.bmp",calib.getImageCount());
			}
			calib.drawChessboardCorners(leftim,leftim,CVEX_STEREO_CALIB_LEFT);
			calib.drawChessboardCorners(rightim,rightim,CVEX_STEREO_CALIB_RIGHT);
			if(is)
			{
				cvexSaveImage(leftim,"calibration_image\\left%03d_.bmp",calib.getImageCount());
				cvexSaveImage(rightim,"calibration_image\\right%03d_.bmp",calib.getImageCount());
			}
		}
		if(key =='p')
		{
			isRectification=true;

			calib.solveStereoParameter();
			calib.showIntrinsicParameters();

			calib.getRectificationMatrix(hleft,hright);
			calib.showRectificationHomography();
		}
		if(key =='h')
		{
			isRectification=true;
			editHomographyGUI(leftim,rightim,hleft,hright);
		}
		if(key =='P')
		{
			isRectification=false;
		}

		if(key =='s')
		{
			static int saveCount=0;
			cvexSaveImage(leftim,"capture\\capture_left%03d.bmp",saveCount);
			cvexSaveImage(rightim,"capture\\capture_right%03d.bmp",saveCount);

			cvWarpPerspective(leftim,render,hleft);
			cvexSaveImage(render,"capture\\capture_left_warp%03d.bmp",saveCount);

			cvWarpPerspective(rightim,render,hright);
			cvexSaveImage(render,"capture\\capture_right_warp%03d.bmp",saveCount++);
		}
		if(key =='F')
		{
			isLRFlip=(isLRFlip)?false:true;
		}

		if(key==CVEX_KEY_ARROW_LEFT)
		{
			mode--;
			cvSetTrackbarPos("switch","3D Webcam",mode);
		}
		if(key==CVEX_KEY_ARROW_RIGHT)
			{
			mode++;
			cvSetTrackbarPos("switch","3D Webcam",mode);
		}

		if(isRectification)
		{
			
			cvWarpPerspective(leftim,render,hleft);
			cvCopy(render,leftim);
			cvWarpPerspective(rightim,render,hright);
			cvCopy(render,rightim);
			/*
			calib.rectifyImageRemap(leftim,leftim,CVEX_STEREO_CALIB_LEFT);
			calib.rectifyImageRemap(rightim,rightim,CVEX_STEREO_CALIB_RIGHT);			
			*/
		}
		else
		{
			cvexWarpShift(rightim,rightim,0,v_shift);
		}
		if(isLRFlip)
		{
			IplImage* swap;
			swap = leftim;
			leftim = rightim;
			rightim=swap;
		}

		switch(mode)
		{
		case MINORU_VIEW_LEFT:
			cvCopy(leftim,render);
			break;
		case MINORU_VIEW_RIGHT:
			cvCopy(rightim,render);
			break;
		case MINORU_VIEW_INTERLACE:
			cvexMakeStereoImageInterlace(leftim,rightim,render,dis-render->width/2);
			break;
		case MINORU_VIEW_SIDEBYSIDE:
			cvexMakeStereoImageSidebySide(leftim,rightim,render,dis-render->width/2);
			break;
		case MINORU_VIEW_DEPTHMAP:
			depth_estimation(leftim,rightim,render,100);
			break;

		default:
		case MINORU_VIEW_ANAGLYPH:
			cvexMakeStereoImageAnaglyph(leftim,rightim,render,dis-render->width/2);
			break;
		}


		{
			//char mess[32];
//			sprintf(mess,"%.02f fps",cTime/AVERAGE_FRAME_NUM_FOR_FPS);
			//cvexPutText(render,mess,cvPoint(50,50),CVEX_WHITE,1);
		}

		cvShowImage ("3D Webcam", render);

		key = cvWaitKey(1);
		
	}

	cvReleaseMat (&hleft);
	cvReleaseMat (&hright);
	cvReleaseImage (&render);
	cvReleaseCapture (&leftcam);
	cvReleaseCapture (&rightcam);
	cvDestroyWindow ("3D Webcam");

}
void simple_template()
{
	CvCapture *leftcam = 0;
	CvCapture *rightcam = 0;
	IplImage *leftim = 0;
	IplImage *rightim = 0;

	leftcam = cvCreateCameraCapture (0);
	rightcam = cvCreateCameraCapture (1);

	double w = IMAGE_WIDTH, h = IMAGE_HEIGHT;
	//camera setup(width and height)
	cvSetCaptureProperty (leftcam, CV_CAP_PROP_FRAME_WIDTH, w);
	cvSetCaptureProperty (leftcam, CV_CAP_PROP_FRAME_HEIGHT, h);
	cvSetCaptureProperty (rightcam, CV_CAP_PROP_FRAME_WIDTH, w);
	cvSetCaptureProperty (rightcam, CV_CAP_PROP_FRAME_HEIGHT, h);

	cvNamedWindow ("3D Webcam", CV_WINDOW_AUTOSIZE);

	leftim = cvQueryFrame (leftcam);
	rightim = cvQueryFrame (rightcam);

	cvShowImage ("3D Webcam", leftim);
	cvWaitKey(0);
	cvShowImage ("3D Webcam", rightim);
	cvWaitKey(0);

	cvReleaseCapture (&leftcam);
	cvReleaseCapture (&rightcam);
	cvDestroyWindow ("3D Webcam");
}

int main (int argc, char **argv)
{
	demo();
	//simple_template();
	return 0;
}