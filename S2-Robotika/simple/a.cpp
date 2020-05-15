#include <iostream>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;


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
}

std::vector<cv::Point> CariRectangle(Mat imgThresholded) {
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Point> points;
    cv::findContours(imgThresholded, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    for (size_t i=0; i<contours.size(); i++) {
        for (size_t j = 0; j < contours[i].size(); j++) {
            cv::Point p = contours[i][j];
            points.push_back(p);
        }
    }
    return points;
//     if (points.size() > 0) {
//         cv::Rect brect = cv::boundingRect(cv::Mat(points).reshape(2));
//         cv::rectangle(imgOriginal, brect.tl(), brect.br(), cv::Scalar(100, 100, 200), 2, CV_AA);
//     }
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

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    //batasTh merah  = {141, 213, 83, 255, 109, 248};
//     batasTh hijau  = {44,  79,  52, 98, 161, 255};
	/*
    batasTh merah = {0,   64,  68, 255, 239, 255};
    batasTh hijau= {44,   87,  108, 255, 121, 255};
    batasTh jingga = {0,   255,  210, 255, 176, 255};
    */
    batasTh merah = {0,   32,  210, 255, 90, 255};
    batasTh hijau= {32,   106,  0, 255, 0, 255};
    batasTh jingga = {95,   162,  0, 255, 0, 255};

    Mat imgThrMerah, imgThrHijau, imgThrJinga;
    
    //batasTh *aktif = &hijau;
    batasTh *aktif = &jingga;
    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &aktif->iLowH, 255); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &aktif->iHighH, 255);

    cvCreateTrackbar("LowS", "Control", &aktif->iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &aktif->iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &aktif->iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &aktif->iHighV, 255);

    while (true) {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video
        //imgOriginal = imread("SamplePict2.png", CV_LOAD_IMAGE_COLOR);
        //bool bSuccess = imread("SamplePict.png", CV_LOAD_IMAGE_COLOR);

        if (!bSuccess) {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }
        if (!imgOriginal.data) {
             cout << "Cannot read a image" << endl;
             break;
        }

        Mat imgHSV;
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        
        
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
        
//         imshow("pre-Thresholded imgThrMerah", imgThrMerah); //show the thresholded image
//         imshow("pre-Thresholded imgThrHijau", imgThrHijau); //show the thresholded image
//         imshow("pre-Thresholded imgThrJinga", imgThrJinga); //show the thresholded image
        std::vector<cv::Point> pointMerah, pointHijau, pointJingga;
        pointMerah = CariRectangle(imgThrMerah);
        pointHijau = CariRectangle(imgThrHijau);
        pointJingga = CariRectangle(imgThrJinga);
        
        if (pointMerah.size() > 0) {
            cv::Rect brect = cv::boundingRect(cv::Mat(pointMerah).reshape(2));
            cv::rectangle(imgOriginal, brect.tl(), brect.br(), cv::Scalar(100, 100, 200), 2, CV_AA);
        }
        if (pointHijau.size() > 0) {
            cv::Rect brect = cv::boundingRect(cv::Mat(pointHijau).reshape(2));
            cv::rectangle(imgOriginal, brect.tl(), brect.br(), cv::Scalar(100, 100, 200), 2, CV_AA);
        }
        if (pointJingga.size() > 0) {
            cv::Rect brect = cv::boundingRect(cv::Mat(pointJingga).reshape(2));
            cv::rectangle(imgOriginal, brect.tl(), brect.br(), cv::Scalar(100, 100, 200), 2, CV_AA);
        }
        
        
        imshow("Original", imgOriginal); //show the original image
//   yea = findBiggestBlob(imgThresholded);
//   imshow("Thresholded Image", imgThresholded); //show the thresholded image

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
