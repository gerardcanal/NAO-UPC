#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <ctype.h>
#include <iostream>
#include <math.h>
#include <time.h>

using namespace cv;
using namespace std;

#define FONT		FONT_HERSHEY_PLAIN
#define FONT_COLOR_DATA	Scalar(255,0,255)
#define LINE_TYPE	CV_AA
#define VIDEOFPS 4
#define VIDEOPATH "video/object_tracking.avi"


int lowerH=0;
int lowerS=146;
int lowerV=40;
int upperH=180;
int upperS=256;
int upperV=256;


//This function threshold the HSV image and create a binary image
Mat GetThresholdedImage(Mat imgHSV)
{
	Mat imgThresh;
	inRange(imgHSV, Scalar(lowerH,lowerS,lowerV), Scalar(upperH,upperS,upperV), imgThresh);
	return imgThresh;
}

//This function tracks the object with bigger area 
void trackObject(Mat imgThresh, Point2f &center, float &area)
{
  //Get the moments
  Moments mu = moments( imgThresh, true ); 

  //Get the mass center:
  Point2f mc;
  if (mu.m00 > 2000){//if area<2000 is noise
  	mc = Point2f( static_cast<float>(mu.m10/mu.m00) , static_cast<float>(mu.m01/mu.m00) ); 
  }else{
  	mc = Point2f(static_cast<float>(0),static_cast<float>(0));
  }
  //printf(" Area (M_00) = %.2f  \n", mu.m00);
  //printf(" POSX = %.2f  POSY = %.2f \n", mc.x, mc.y);
  area = mu.m00;
  center = mc;
}

int main(){
	VideoCapture cap(0); // open the default camera
    	if(!cap.isOpened()){
		printf("Capture failure\n"); 
        	return -1;
	}

	namedWindow("Tomatoe", CV_WINDOW_KEEPRATIO);

	//Frame rate variables
	time_t start,end;
	time(&start);
	int counterTime=0;

	//iterate through each frames of the video
	Mat frame;
	cap >> frame;
	VideoWriter outputVideo(VIDEOPATH, CV_FOURCC('D','I','V','X'), VIDEOFPS, frame.size(), true);
	while(true){
		cap >> frame;

		Mat imgHSV;
		cvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV

		Mat imgThresh = GetThresholdedImage(imgHSV);

		blur( imgThresh, imgThresh, Size(3,3) );

		Point2f obj_pos;
		float area;
		trackObject(imgThresh, obj_pos, area);
		circle(frame, obj_pos, 10, Scalar(20,237,0), -1 , 8, 0);
		circle(frame, obj_pos, sqrt(area/M_PI), Scalar(20,237,0), 2, 8, 0);
		
		//Print frame rate
		time(&end);
		++counterTime;
		double sec=difftime(end,start);
		double fps=counterTime/sec;
		putText(frame, format("FPS: %lf",fps), cvPoint(frame.cols-180, 40), FONT, 2, FONT_COLOR_DATA, 1.5, LINE_TYPE);

		imshow("Tomatoe", frame);
		outputVideo.write(frame);
		cap >> frame;

		//Wait 80mS
		int c = cvWaitKey(80);
		//If 'ESC' is pressed, break the loop
		if((char)c==27 ) break;
	}
	return 0;
}
