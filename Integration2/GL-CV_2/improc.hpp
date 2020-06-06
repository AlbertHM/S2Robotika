#include <iostream>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

typedef struct {
    int iLowH;
    int iHighH;
    int iLowS;
    int iHighS;
    int iLowV;
    int iHighV;
} batasTh;

typedef struct{
	int d_l = 0; // Detect Link
	int d_o = 0; // Detect Object
	double deg_link; // Degree Link
	double deg_obj; // Degree Object
} dataImProc;

batasTh merah = { 48,  93, 136, 255,  85, 255}; // Objek
batasTh hijau = { 87, 122, 155, 255, 132, 255}; // End-effector
batasTh jingga = {147, 195, 160, 255, 170, 255}; // Base

vector<Point> CariRectangle(Mat imgThresholded) 
{
    int largest_area=0;
    int largest_contour_index=0;
    
    vector<vector<Point> > contours; // Vector for storing contour
    vector<Point> points;
    vector<Vec4i> hierarchy;
    
    cv::findContours(imgThresholded, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    
    // Find biggest blob
    if(contours.size()>0)
    {
		// THIS IF solve segmentation error, core dumped
		// When object not detected, there is nothing to push_back, lead to error
		for( int i = 0; i< contours.size(); i++ ) 		// Iterate through each contour. 
		{
			double a=contourArea( contours[i],false);	// Find the area of contour
			if(a>largest_area)
			{
				largest_area=a;
				largest_contour_index=i;                // Store the index of largest contour
			}
		}
		for (size_t j = 0; j < contours[largest_contour_index].size(); j++)
		{
			cv::Point p = contours[largest_contour_index][j];
			points.push_back(p);
		}
	}
    return points;
}

void Improc(Mat imgOriginal, dataImProc *dataip)
{
	Mat imgHSV, imgThrMerah, imgThrHijau, imgThrJinga;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	
	batasTh *aktif;
	aktif = &merah;
	inRange(imgHSV, Scalar(aktif->iLowH, aktif->iLowS, aktif->iLowV), Scalar(aktif->iHighH, aktif->iHighS, aktif->iHighV), imgThrMerah); //Threshold the image
	aktif = &hijau;
	inRange(imgHSV, Scalar(aktif->iLowH, aktif->iLowS, aktif->iLowV), Scalar(aktif->iHighH, aktif->iHighS, aktif->iHighV), imgThrHijau); //Threshold the image
	aktif = &jingga;
	inRange(imgHSV, Scalar(aktif->iLowH, aktif->iLowS, aktif->iLowV), Scalar(aktif->iHighH, aktif->iHighS, aktif->iHighV), imgThrJinga); //Threshold the image
	/*
	imshow("Thresholded imgThrMerah", imgThrMerah); //show the thresholded image
	imshow("Thresholded imgThrHijau", imgThrHijau); //show the thresholded image
	imshow("Thresholded imgThrJinga", imgThrJinga); //show the thresholded image
	*/
	erode(imgThrMerah,  imgThrMerah, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate(imgThrMerah, imgThrMerah, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	dilate(imgThrMerah, imgThrMerah, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThrMerah,  imgThrMerah, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	
	erode(imgThrHijau,  imgThrHijau, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate(imgThrHijau, imgThrHijau, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	dilate(imgThrHijau, imgThrHijau, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThrHijau,  imgThrHijau, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	
	erode(imgThrJinga,  imgThrJinga, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate(imgThrJinga, imgThrJinga, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	dilate(imgThrJinga, imgThrJinga, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThrJinga,  imgThrJinga, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		
	std::vector<cv::Point> pointMerah, pointHijau, pointJingga;	
	
	pointMerah = CariRectangle(imgThrMerah);
	pointHijau = CariRectangle(imgThrHijau);
	pointJingga = CariRectangle(imgThrJinga);	
	
	Point mcMerah, mcHijau, mcJingga;
	
	int detectStat[3] = {0,0,0}; // Objek, End-E, Base
	
	// Objek
	if (pointMerah.size() > 0) {			
		cv::Rect brect = cv::boundingRect(cv::Mat(pointMerah).reshape(2));
		cv::rectangle(imgOriginal, brect.tl(), brect.br(), cv::Scalar(100, 100, 200), 2, CV_AA);
		
		mcMerah.x = brect.x + (brect.width/2);
		mcMerah.y = brect.y + (brect.height/2);
		circle(imgOriginal, mcMerah, 4, Scalar(20,20,20));
		
		ostringstream sample;
		sample << "Objek " << mcMerah.x << "," << mcMerah.y;
		string text = sample.str();
		putText(imgOriginal,text, mcMerah, FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),1);
		
		detectStat[0] = 1;
	}
	// End-effector
	if (pointHijau.size() > 0) {
		cv::Rect brect = cv::boundingRect(cv::Mat(pointHijau).reshape(2));
		cv::rectangle(imgOriginal, brect.tl(), brect.br(), cv::Scalar(100, 100, 200), 2, CV_AA);
		
		mcHijau.x = brect.x + (brect.width/2);
		mcHijau.y = brect.y + (brect.height/2);
		circle(imgOriginal, mcHijau, 4, Scalar(20,20,20));
		
		ostringstream sample;
		sample << "End-E " << mcHijau.x << "," << mcHijau.y;
		string text = sample.str();
		putText(imgOriginal,text, mcHijau, FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),1);
		
		detectStat[1] = 1;
	}
	// Base
	if (pointJingga.size() > 0) {
		cv::Rect brect = cv::boundingRect(cv::Mat(pointJingga).reshape(2));
		cv::rectangle(imgOriginal, brect.tl(), brect.br(), cv::Scalar(100, 100, 200), 2, CV_AA);
		
		mcJingga.x = brect.x + (brect.width/2);
		mcJingga.y = brect.y + (brect.height/2);
		circle(imgOriginal, mcJingga, 4, Scalar(20,20,20));
		
		ostringstream sample;
		sample << "Base " << mcJingga.x << "," << mcJingga.y;
		string text = sample.str();
		putText(imgOriginal,text, mcJingga, FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),1);
		
		detectStat[2] = 1;
	}
	double result;
	double theta_obj_base;
	
	// Draw robot link
	if(detectStat[1] && detectStat[2])
	{
		line(imgOriginal, mcJingga, mcHijau, Scalar(100,100,200), 4);

		Point delta, posText;
		double param;//, result;
		delta.x = mcHijau.x-mcJingga.x;
		delta.y = mcHijau.y-mcJingga.y;
		param = (-1.0*delta.y)/(1.0*delta.x);
		if(mcHijau.x == mcJingga.x)
		{
			result = 90.0;
		}
		else if(mcHijau.x > mcJingga.x)
		{
			result = atan(param) * 180 / PI;
		}
		else
		{
			result = atan(param) * 180 / PI + 180;
		}
		
		dataip->deg_link = result;
		
		//printf("||%d ++ %d {%.2f}{%.2f}||",delta.y, delta.x, param, result);
		posText.x = mcJingga.x + delta.x/2;
		posText.y = mcJingga.y + delta.y/2;
		
		ostringstream sample;
		sample << result <<"*";
		string text = sample.str();
		putText(imgOriginal,text, posText, FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),1);
	}
	
	// Draw line base-object
	if(detectStat[0] && detectStat[2])
	{
		line(imgOriginal, mcJingga, mcMerah, Scalar(100,100,200), 4);

		Point delta, posText;
		double param;
		delta.x = mcMerah.x-mcJingga.x;
		delta.y = mcMerah.y-mcJingga.y;
		param = (-1.0*delta.y)/(1.0*delta.x);
		if(mcMerah.x == mcJingga.x)
		{
			theta_obj_base = 90.0;
		}
		else if(mcMerah.x > mcJingga.x)
		{
			theta_obj_base = atan(param) * 180 / PI;
		}
		else
		{
			theta_obj_base = atan(param) * 180 / PI + 180;
		}
		//printf("||%d ++ %d {%.2f}{%.2f}||",delta.y, delta.x, param, result);
		posText.x = mcJingga.x + delta.x/2;
		posText.y = mcJingga.y + delta.y/2;
		
		ostringstream sample;
		sample << theta_obj_base <<"*";
		string text = sample.str();
		putText(imgOriginal,text, posText, FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),1);
	}
	
	Point posText;
	int spacing = 15;
	posText.x = 0;
	posText.y = spacing;
	ostringstream sample;
	sample << "Base" << detectStat[2];
	string text = sample.str();
	putText(imgOriginal,text, posText, FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),1);
	posText.y += spacing;
	
	sample.str(""); // reset string to empty
	sample.clear(); // clear any error flag that may set
	sample << "End effector" << detectStat[1];
	text = sample.str();
	putText(imgOriginal,text, posText, FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),1);
	posText.y += spacing;
	
	sample.str("");
	sample.clear();
	sample << "Object" << detectStat[0];
	text = sample.str();
	putText(imgOriginal,text, posText, FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),1);
	posText.y += spacing;
	
	sample.str("");
	sample.clear();
	sample << "Theta" << result <<"*";
	text = sample.str();
	putText(imgOriginal,text, posText, FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),1);
	posText.y += spacing;
	
	namedWindow( "Original", WINDOW_AUTOSIZE );
	imshow("Original", imgOriginal);
	waitKey(1);
}
