#include <iostream>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>

using namespace cv;
using namespace std;

#define PI 3.14159265

/*
Mat findBiggestBlob(cv::Mat & matImage){
    int largest_area=0;
    int largest_contour_index=0;

    vector< vector<Point> > contours; // Vector for storing contour
    vector<Vec4i> hierarchy;

    findContours( matImage, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image

    for( int i = 0; i< contours.size(); i++ ) {// iterate through each contour. 
        double a=contourArea( contours[i],false);  //  Find the area of contour
        if(a>largest_area){
            largest_area=a;
            largest_contour_index=i;                //Store the index of largest contour
            //bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
        }
    }

    drawContours( matImage, contours, largest_contour_index, Scalar(255), CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
    return matImage;
}*/

std::vector<cv::Point> CariRectangle(Mat imgThresholded) {
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
		//cout << "1" << endl;
		for( int i = 0; i< contours.size(); i++ ) {// iterate through each contour. 
			double a=contourArea( contours[i],false);  //  Find the area of contour
			if(a>largest_area){
				largest_area=a;
				largest_contour_index=i;                //Store the index of largest contour
			}
		}
		//cout << "2" << endl;
		for (size_t j = 0; j < contours[largest_contour_index].size(); j++) {
			cv::Point p = contours[largest_contour_index][j];
			points.push_back(p);
		}
	}
	
	//cout << "3" << endl;
    return points;
}

typedef struct {
    int iLowH;
    int iHighH;
    int iLowS;
    int iHighS;
    int iLowV;
    int iHighV;
} batasTh;
bool servoSistemAktif = false;

int main( int argc, char** argv )
 {
	 
    VideoCapture cap(0); //capture the video from web cam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    //namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    batasTh merah = { 48,  93, 136, 255,  85, 255}; // Objek
    batasTh hijau= { 87, 122, 155, 255, 132, 255}; // End-effector
    batasTh jingga = {147, 195, 160, 255, 170, 255}; // Base

    Mat imgThrMerah, imgThrHijau, imgThrJinga;
    
    /*
    //batasTh *aktif = &hijau;
    batasTh *aktif = &jingga;
    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &aktif->iLowH, 255); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &aktif->iHighH, 255);

    cvCreateTrackbar("LowS", "Control", &aktif->iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &aktif->iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &aktif->iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &aktif->iHighV, 255);*/

    while (true) {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video
        //imgOriginal = imread("SamplePict2.png", CV_LOAD_IMAGE_COLOR);
        //bool bSuccess = imread("SamplePict.png", CV_LOAD_IMAGE_COLOR);

        if (!bSuccess) {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }
		/*
        if (!imgOriginal.data) {
             cout << "Cannot read a image" << endl;
             break;
        }*/

        Mat imgHSV;
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        //cout << "Here0" << endl;
        
        batasTh *aktif;
        aktif = &merah;
        inRange(imgHSV, Scalar(aktif->iLowH, aktif->iLowS, aktif->iLowV), Scalar(aktif->iHighH, aktif->iHighS, aktif->iHighV), imgThrMerah); //Threshold the image
        aktif = &hijau;
        inRange(imgHSV, Scalar(aktif->iLowH, aktif->iLowS, aktif->iLowV), Scalar(aktif->iHighH, aktif->iHighS, aktif->iHighV), imgThrHijau); //Threshold the image
        aktif = &jingga;
        inRange(imgHSV, Scalar(aktif->iLowH, aktif->iLowS, aktif->iLowV), Scalar(aktif->iHighH, aktif->iHighS, aktif->iHighV), imgThrJinga); //Threshold the image
      
        imshow("Thresholded imgThrMerah", imgThrMerah); //show the thresholded image
        imshow("Thresholded imgThrHijau", imgThrHijau); //show the thresholded image
        imshow("Thresholded imgThrJinga", imgThrJinga); //show the thresholded image
        
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
        //cout << "Here1" << endl;
        
        std::vector<cv::Point> pointMerah, pointHijau, pointJingga;
        //cout << "Here1.5" << endl; // error at 1.5 ~ 2
        // apabila tidak didapatkan objek, dapat menyebabkan error Sehmentation error, Core dumped
        
        pointMerah = CariRectangle(imgThrMerah);
        pointHijau = CariRectangle(imgThrHijau);
        pointJingga = CariRectangle(imgThrJinga);
        //cout << "Here2" << endl;
        
        Point mcMerah, mcHijau, mcJingga;
        //cout << "Here10" << endl;
        int detectStat[3] = {0,0,0};
        //cout << "Here11" << endl;
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
        // Draw link
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
			//printf("||%d ++ %d {%.2f}{%.2f}||",delta.y, delta.x, param, result);
			posText.x = mcJingga.x + delta.x/2;
			posText.y = mcJingga.y + delta.y/2;
			
            ostringstream sample;
            sample << result <<"*";
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
		
        
        imshow("Original", imgOriginal);

        char c = cvWaitKey(10);
        switch (c) {
            case 'A': {
                servoSistemAktif = true;
                printf("Servomechanism Aktif\r\n");
                break;
            }
            case 'a': {
                servoSistemAktif = false;
                printf("Servomechanism Mati\r\n");
                break;
            }
        }
        if (waitKey(30) == 27) {
            cout << "esc key is pressed by user" << endl;
            break; 
       }
    }

   return 0;

} 
