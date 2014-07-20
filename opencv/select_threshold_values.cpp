#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <ctype.h>
#include <iostream>


using namespace cv;
using namespace std;

int lowerH=0;
int lowerS=0;
int lowerV=0;
int upperH=0;
int upperS=0;
int upperV=0;

int maxH=180;
int maxS=256;
int maxV=256;


//This function threshold the HSV image and create a binary image
Mat GetThresholdedImage(Mat imgHSV){
	Mat imgThresh;
	inRange(imgHSV, Scalar(lowerH,lowerS,lowerV), Scalar(upperH,upperS,upperV), imgThresh);
	return imgThresh;
}

//This function create two windows and 6 trackbars for the "Ball" window
void setTrackbars(){

	createTrackbar("LowerH", "Ball", &lowerH, maxH, NULL);
	        createTrackbar("UpperH", "Ball", &upperH, maxH, NULL);

	createTrackbar("LowerS", "Ball", &lowerS, maxS, NULL);
	        createTrackbar("UpperS", "Ball", &upperS, maxS, NULL);

	createTrackbar("LowerV", "Ball", &lowerV, maxV, NULL);
	        createTrackbar("UpperV", "Ball", &upperV, maxV, NULL);
}

int main(){
	VideoCapture cap(0); // open the default camera
    	if(!cap.isOpened()){
		printf("Capture failure\n"); 
        	return -1;
	}

	namedWindow("Video");
	namedWindow("Ball", CV_WINDOW_KEEPRATIO);
	Mat frame;
	setTrackbars();
	//iterate through each frames of the video
	while(true){
		cap >> frame;
		Mat imgHSV;
		cvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
		Mat imgThresh = GetThresholdedImage(imgHSV);
		imshow("Ball", imgThresh);
		//imshow("Video", frame);

		//Wait 80mS
		int c = cvWaitKey(80);
		//If 'ESC' is pressed, break the loop
		if((char)c==27 ) break;
	}
	return 0;
}
