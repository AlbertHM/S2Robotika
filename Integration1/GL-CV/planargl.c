/////////////////////////////////////////////////////////////
/* Template OpengGL sengaja dibuat untuk kuliah robotik 
*  di Departemen Teknik Elektro
*  Bagi yang ingin memodifikasi untuk keperluan yang lain,
*  dipersilahkan dengan menuliskan acknowledgement pada
*    Dr. Abdul Muis, MEng.
*    Autonomous Control Electronics (ACONICS) Research Group
*    http://www.ee.ui.ac.id/aconics
*////////////////////////////////////////////////////////////

#include <stdio.h> 
#include <iostream>
#include <stdlib.h> 
#include <GL/glut.h> // Header File For The GLUT Library
#include <GL/gl.h> // Header File For The OpenGL32 Library
#include <GL/glu.h> // Header File For The GLu32 Library
#include <unistd.h> // Header file for sleeping.
#include <math.h> 
#include <fcntl.h>			/* File control definitions */
#include <errno.h>			/* Error number definitions */
#include <termios.h>		/* POSIX terminal control definitions */
#include <sys/time.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "planar.c"

using namespace cv;
using namespace std;

#define PI 3.14159265

/* ascii code for the escape key */
#define ESCkey	27

/* The number/handle of our GLUT window */
int window, wcam;  
 
FILE *ff;
FILE *gpdata;

/* To draw a quadric model */
GLUquadricObj *obj;

// ROBOT MODEL PARAMATER
#define Xoffset	0.0	
#define Yoffset	0.0
#define Zoffset	0.2
#define img_height 200
#define img_width 350
#define threshold 230

#define Link1 L1
#define Link2 L2

unsigned char send1;
unsigned char send2;
unsigned char header;

float *tetha1=&q1; //tetha dalam radian
float *tetha2=&q2;
float *x=&objx;
float *y=&objy;

char debug=0;

int gerak = 0;
int counter = 0;
int TS = 0;

//Param robot
float l[2] = {1,1};

//Param Task Space
//best so far 0.2 & 0.1
float Kp = 0.2;
float Kd = 0.1;

float Period = 3;
float step = 50 * Period;

float xr[2], xawal[2], xakhir[2],xcmd[2],xcmdold[2];
float dxcmd[2], dxcmdold[2], ddxcmd[2];
float q[2];
float dq[2], ddq[2];
float Jb1[2], Jb2[2], iJb1[2], iJb2[2];
float det;
float dt = 0.02;


void Sim_main(void); // Deklarasi lebih awal agar bisa diakses oleh fungsi sebelumnya
void display(void); // fungsi untuk menampilkan gambar robot / tampilan camera awal
void camera_result(void); // fungsi untuk menampilkan hasil olah camera

// Hasil gambar titik awal di pojok kiri bawah
/*  | gambar[img_height][0]
*   |
*   |
*   |
*   |
*   |
*   |
*   |
*   |
*   x________________________ gambar[0][img_width]
*
*///////////////////////////////////////////////////

unsigned char gambarGray[img_height+2][img_width+2]; // pixel untuk menampung gambar grayscale
unsigned char gambarR[img_height+2][img_width+2]; // pixel untuk menampung gambar merah
unsigned char gambarG[img_height+2][img_width+2]; // pixel untuk menampung gambar hijau
unsigned char gambarB[img_height+2][img_width+2]; // pixel untuk menampung gambar biru


  
/* define color */  
GLfloat green1[4]  ={0.8, 1.0, 0.8, 1.0};
GLfloat blue1[4]  ={0.1, 0.1, 1.0, 1.0};
GLfloat blue2[4]  ={0.2, 0.2, 1.0, 1.0};
GLfloat blue3[4]  ={0.3, 0.3, 1.0, 1.0};
GLfloat yellow1[4]={0.1, 0.1, 0.0, 1.0};
GLfloat yellow2[4]={0.2, 0.2, 0.0, 1.0};
GLfloat pink6[4] ={0.8, 0.55, 0.6, 1.0};
GLfloat yellow5[4]={0.8, 0.8, 0.0, 1.0};
GLfloat abu2[4]={0.5,0.5,0.5,1.0};

// Image Processing

VideoCapture cap;
int deviceID = 0;
//int apiID = cv::CAP_ANY;

typedef struct {
    int iLowH;
    int iHighH;
    int iLowS;
    int iHighS;
    int iLowV;
    int iHighV;
} batasTh;

batasTh merah = { 48,  93, 136, 255,  85, 255}; // Objek
batasTh hijau= { 87, 122, 155, 255, 132, 255}; // End-effector
batasTh jingga = {147, 195, 160, 255, 170, 255}; // Base

vector<Point> CariRectangle(Mat imgThresholded) {
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


void  drawOneLine(double x1, double y1, double x2, double y2) 
   {glBegin(GL_LINES); glVertex3f((x1),(y1),0.0); glVertex3f((x2),(y2),0.0); glEnd();}
   
void  model_cylinder(GLUquadricObj * object, GLdouble lowerRadius, GLdouble upperRadius, GLdouble length, GLint res, GLfloat *color1, GLfloat *color2)
{
  glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glTranslatef(0,0,-length/2);
	  gluCylinder(object, lowerRadius, upperRadius, length, 20, res);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    gluDisk(object, 0.01, lowerRadius, 20, res); 
    glTranslatef(0, 0, length);
    gluDisk(object, 0.01, upperRadius, 20, res); 
  glPopMatrix();
}

void disp_floor(void)
{
  int i,j,flagc=1;

  glPushMatrix();
  
  GLfloat dx=4.5,dy=4.5;
  GLint amount=15;
  GLfloat x_min=-dx/2.0, x_max=dx/2.0, x_sp=(GLfloat) dx/amount, y_min=-dy/2.0, y_max=dy/2.0, y_sp=(GLfloat) dy/amount;

  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green1);
  for(i = 0; i<=48; i++){
     drawOneLine(-2.4+0.1*i, -2.4,       -2.4+0.1*i,  2.4);
     drawOneLine(-2.4,       -2.4+0.1*i,  2.4,       -2.4+0.1*i);
  }

  glPopMatrix();
}

void  lighting(void)
{

	GLfloat light_ambient[] =  {0.2, 0.2, 0.2, 1.0};
	GLfloat light_diffuse[] =  {0.4, 0.4, 0.4, 1.0};
	GLfloat light_specular[] = {0.3, 0.3, 0.3, 1.0};
	GLfloat light_position[] = {2, 0.1, 7,1.0};
	GLfloat spot_direction[] = {0.0, -0.1, -1.0, 1.0};

	glClearColor(0.0, 0.0, 0.0, 0.0);     
  
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 40.0);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, spot_direction);
	glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 4);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
}

void disp_robot(void)
{
  glPushMatrix();
    glTranslatef(Xoffset, Yoffset, Zoffset/2);
    // Draw base
    model_cylinder(obj, 0.1, 0.1, Zoffset, 2, abu2, abu2);
    // Menuju joint-1
    glTranslatef(0, 0, Zoffset/2);
    glRotatef(*tetha1*RTD,0,0,1);
    glPushMatrix();
      // Gambar link1
      glRotatef(-90,1,0,0);
      glTranslatef(0,0,Link1/2);
      model_cylinder(obj, 0.03, 0.03, Link1, 2, pink6, yellow2);
    glPopMatrix();
    // Menuju joint-2
    glTranslatef(0,Link1, 0);
    glRotatef(*tetha2*RTD,0,0,1);
    glPushMatrix();
      glDisable(GL_LIGHTING);
      glColor3f(0.0,1.0,0.0);  // buat tanda hijau (R G B) = 0 1 0             
      glTranslatef(0,0,0.04); // pindahkan offset 1cm diatas sendi 2
      gluDisk(obj, 0.001, 0.01, 20, 2); // bikin tanda lingkaran merah diatas sendi 2 
      glEnable(GL_LIGHTING);
    glPopMatrix();
    glPushMatrix();
      // Gambar link2
      glRotatef(-90,1,0,0);
      glTranslatef(0,0,Link2/2);
      model_cylinder(obj, 0.03, 0.03, Link2, 2, yellow5, yellow2);
    glPopMatrix();
    glPushMatrix();
      glDisable(GL_LIGHTING);
      glColor3f(0.0,0.0,1.0);  // buat tanda biru (R G B) = 0 0 1             
      glTranslatef(0,Link2,0.04); // pindahkan offset 1cm diatas sendi 2
      gluDisk(obj, 0.001, 0.01, 20, 2); // bikin tanda lingkaran merah diatas sendi 2 
      glEnable(GL_LIGHTING);
    glPopMatrix();
  glPopMatrix();
  glPushMatrix();
    glTranslatef(*x, *y, Zoffset/2);
    // Draw obj
    model_cylinder(obj, 0.03, 0.03, Zoffset, 2, yellow5, abu2);
    
    glDisable(GL_LIGHTING);
    glColor3f(1.0,0.0,0.0); // buat tanda merah (R G B) = 1 0 0 
    glTranslatef(0,0,Zoffset/2+0.01); // pindahkan offset 1cm diatas object
    gluDisk(obj, 0.001, 0.01, 20, 2); // bikin tanda lingkaran merah diatas object 
    glEnable(GL_LIGHTING);

  glPopMatrix();

}

// Draw Object
void display(void)
{
//   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   //glLoadIdentity();  // Reset View
   disp_floor();
   //hitung_robot();
   disp_robot();

   /* since window is double buffered, 
      Swap the front and back buffers (used in double buffering). */
   glutSwapBuffers() ; 
   
   // Transfer warna jadi grayscale, dan di buat masing2 0.15 scalanya (seharusnya 0.3333333)
   glPixelTransferf(GL_RED_SCALE,0.15);
	 glPixelTransferf(GL_GREEN_SCALE,0.15);
	 glPixelTransferf(GL_BLUE_SCALE,0.15); 
	 glReadPixels(0,0, img_width,img_height, GL_LUMINANCE,GL_UNSIGNED_BYTE, gambarGray);
	 
	 // ambil warna merah saja
	 glPixelTransferf(GL_RED_SCALE,1);
	 glPixelTransferf(GL_GREEN_SCALE,0);
	 glPixelTransferf(GL_BLUE_SCALE,0); 
	 glReadPixels(1,1,img_width,img_height, GL_LUMINANCE,GL_UNSIGNED_BYTE, gambarR);
	 // ambil warna hijau saja
	 glPixelTransferf(GL_RED_SCALE,0);
	 glPixelTransferf(GL_GREEN_SCALE,1);
	 glPixelTransferf(GL_BLUE_SCALE,0); 
	 glReadPixels(0,0, img_width,img_height, GL_LUMINANCE,GL_UNSIGNED_BYTE, gambarG);
	 // ambil warna biru saja
	 glPixelTransferf(GL_RED_SCALE,0);
	 glPixelTransferf(GL_GREEN_SCALE,0);
	 glPixelTransferf(GL_BLUE_SCALE,1); 
	 glReadPixels(0,0, img_width,img_height, GL_LUMINANCE,GL_UNSIGNED_BYTE, gambarB);

}

//Task Space Control
void forward_kinematic()
{
	xr[0] = l[0]*cos(*tetha1) + l[1]*cos(*tetha1+*tetha2);
	xr[1] = l[0]*sin(*tetha1) + l[1]*sin(*tetha1+*tetha2);
}

void trajectory_init()
{	
	forward_kinematic();
	
	for(int u=0; u<2; u++)
	{
		xawal[u] = xr[u];
		dxcmd[u] = 0;
		dxcmdold[u] = 0;
		ddxcmd[u] = 0;
	}
}

void trajectory_plan()
{
	for(int u=0;u<2;u++)
	{
		xcmd[u] = xawal[u] + (xakhir[u] - xawal[u]) * counter/step;
	}
}

void derivative()
{
	for(int u=0;u<2;u++)
	{
		dxcmd[u] = (xcmd[u] - xr[u]);
		ddxcmd[u] = (dxcmd[u]-dxcmdold[u]);
	}	
}

void PDControl()
{
	for(int u=0;u<2;u++)
	{
		ddxcmd[u] = Kp * dxcmd[u] + Kd * ddxcmd[u];
	}
}

void inverse_jacobian()
{	
	Jb1[0] = -xr[1];
	Jb1[1] = -l[1]*sin(*tetha1+*tetha2);
	Jb2[0] = xr[0];
	Jb2[1] = l[1]*cos(*tetha1+*tetha2);
	det = Jb1[0]*Jb2[1]-Jb2[0]*Jb1[1] ;
	//printf("%f | %f | %f | %f\n", Jb1[0], Jb1[1], Jb2[0], Jb2[1]);
	//printf("%f",det);
	if(abs(det) < 0.001)
	{
		//printf("Determinan : %f", det);
	}
	iJb1[0] = Jb2[1] / det;
	iJb1[1] = -Jb1[1] /det;
	iJb2[0] = -Jb2[0] /det;
	iJb2[1] = Jb1[0] /det;
	
	ddq[0] = iJb1[0]*ddxcmd[0] + iJb1[1]*ddxcmd[1];
	ddq[1] = iJb2[0]*ddxcmd[0] + iJb2[1]*ddxcmd[1];
}

void integral()
{
	for(int u=0; u<2; u++)
	{
		dq[u] += ddq[u];
		q[u] += dq[u];
	}
}

void memory()
{
	for(int u=0;u<2;u++)
	{
		dxcmdold[u] = dxcmd[u];
	}	
}

void Sim_main(void)
{
	unsigned long Xr=0,Yr=0, Xg=0,Yg=0, Xb=0,Yb=0; // titik untuk menghitung sum
	int Nr=0, Ng=0, Nb=0;
	static unsigned int Rx,Ry, Gx,Gy, Bx,By; // untuk menyimpan hasil titik berat
	unsigned int i,j,k;
	glutSetWindow(window);
	
	// ========================= Image Processing Section ==================================== //
	
	Mat imgOriginal;

	bool bSuccess = cap.read(imgOriginal); 
	//imgOriginal = imread("SamplePict2.png", CV_LOAD_IMAGE_COLOR);	
	if (!bSuccess) 
	{
		 cout << "Cannot read a frame from video stream" << endl;
	}
	
	Mat imgHSV, imgThrMerah, imgThrHijau, imgThrJinga;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	//cout << "Here0" << endl;
	
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
	
	namedWindow( "Original", WINDOW_AUTOSIZE );
	imshow("Original", imgOriginal);
	waitKey(1);
	
	
	// ========================= Image Processing Section ==================================== //
	
	
	forward_kinematic();
	if((xr[0] != xakhir[0]) || (xr[1] != xakhir[1]))
	{	
		q[0] = *tetha1;
		q[1] = *tetha2;
		//printf("%f\n",xr[0]);
		//printf("Theta %f || %f \n", q[0]*RTD, q[1]*RTD);
		forward_kinematic();
		printf("[%d]Xr %f || %f \n", counter, xr[0], xr[1]);
		if(counter!=step)
		{
			counter++;
		}
		trajectory_plan();
		//printf("%f || %f \n", xcmd[0], xcmd[1]); //ok
		derivative();
		//printf("%f || %f \n", ddxcmd[0], ddxcmd[1]); //ok
		PDControl();
		//printf("%f || %f \n", ddxcmd[0], ddxcmd[1]); //masih ok
		inverse_jacobian();
		//printf("%f || %f \n", ddq[0], ddq[1]); // sudah dibetulkan!
		integral();
		memory();
		*tetha1=q[0];
		*tetha2=q[1];
		fprintf(ff,"%d, %.4f, %.4f, %.4f, %.4f, \n", counter, xcmd[0], xr[0], xcmd[1], xr[1]);
		fprintf(gpdata,"%d, %.3f, %.3f, %.3f, %.3f, \n", counter, xcmd[0], xr[0], xcmd[1], xr[1]);
		if(counter == step)
		{
			xr[0] = xakhir[0];
			xr[1] = xakhir[1];
		}
		sleep(0.02);
	}
  
	
	header = 0xF5;
	send1 = *tetha1*RTD + 90;
	send2 = *tetha2*RTD + 90;
	printf("## %u ## %u ##\n",(unsigned)send1,(unsigned)send2);
	/*
	if(send1 < 0)
	{
		send1 = 0;
	}
	if(send1 > 180)
	{
		send1 = 180;
	}
	if(send2 < 0)
	{
		send2 = 0;
	}
	if(send2 > 180)
	{
		send2 = 180;
	}
	*/
	
	/*
	printf("## %u ## %u ##",(unsigned)send1,(unsigned)send2);
	write(fd,&header,sizeof(header));//header
	write(fd,&send1,sizeof(send1));//data sudut 1
	write(fd,&send2,sizeof(send2));//data sudut 1*/
	
  
  display();
  
  // Cari titik berat Titik titik feature
  for (i=0;i<img_width;i++)
   for (j=0;j<img_height;j++) {
   	  // Tes Treshold
   	  if (gambarR[j][i]>threshold) {Xr+=i;Yr+=j;Nr++;
   	  	  if (debug) printf("%d,%d ",i,j);}
   	  if (gambarG[j][i]>threshold) {Xg+=i;Yg+=j;Ng++;}
   	  if (gambarB[j][i]>threshold) {Xb+=i;Yb+=j;Nb++;}
   }
  if (debug) {
  	printf("\n");
  	for (k=0;k<img_width*img_height;k++)
  	  if (*(&gambarR[0][0]+k)>threshold) printf("%d ",k);
  	printf("\n");
  	
  }
  // Hitung titik berat 
  if (Nr) {Rx=Xr/Nr;  Ry=Yr/Nr;}
  if (Ng) {Gx=Xg/Ng;  Gy=Yg/Ng;}
  if (Nb) {Bx=Xb/Nb;  By=Yb/Nb;}
  //Display hasil extract ke gambarGray untuk ditampilkan
  gambarGray[Ry][Rx]=255;
  gambarGray[Gy][Gx]=255;
  gambarGray[By][Bx]=255;
  
  //printf("=%d %d %d %d %d %d\n",Rx, Ry, Gx, Gy, Bx, By);
  
  glutSetWindow(wcam);
  camera_result(); 	
	
}

void keyboard(unsigned char key, int i, int j)
{
	 switch(key){
      case ESCkey: fclose(ff); fclose(gpdata); exit(1); break;
      case '1': *x=*x+0.01; break;
      case '2': *x=*x-0.01; break;
      case '5': *y=*y+0.01; break;
      case '6': *y=*y-0.01; break;
	  case 'A': *tetha1+=10*DTR; break;
	  case 'a': *tetha1-=10*DTR; break;
	  case 'F': *tetha2+=10*DTR; break;
	  case 'f': *tetha2-=10*DTR; break;
	  case 'H': gerak = 1; break;
	  case 'h': gerak = 0; break;
	  //case 'z': TS = 1; xfinal[0] = x[0]+0.1; xfinal[1] = x[1]+0.1; break;
	  case 'z': trajectory_init(); xakhir[0] = xr[0]-0.1; xakhir[1] = xr[1]-0.1; counter = 0; break;
      //case 'd': debug=(~debug) & 0x1; break;
      //case 's': glutIdleFunc(NULL); break;
      //case 'r': glutIdleFunc(&Sim_main); break;
      
   }
}

void camera_result(void)
{
   glClear(GL_COLOR_BUFFER_BIT);
   if (debug) 
   	 glDrawPixels(img_width, img_height, GL_LUMINANCE, GL_UNSIGNED_BYTE, gambarR);
   else
   	 glDrawPixels(img_width, img_height, GL_LUMINANCE, GL_UNSIGNED_BYTE, gambarGray);

   glutSwapBuffers();  
}

void camera_window(void)
{
	/*----------Camera Window----------*/
    //glutInitDisplayMode(GLUT_DOUBLE |  GLUT_DEPTH);

	 glutInitWindowSize(img_width,img_height);		
   glutInitWindowPosition (500, 100);
	 wcam=glutCreateWindow("Camera Process");
   glClearColor(0.0f, 0.0f, 1.0f, 1.0f); 
   glutDisplayFunc (&camera_result) ;
   glutKeyboardFunc(&keyboard);
}


void init(void) 
{ 
   obj = gluNewQuadric(); 
   /* Clear background to (Red, Green, Blue, Alpha) */
   glClearColor(0.0f, 0.0f, 0.0f, 0.0f) ;
   glEnable(GL_DEPTH_TEST); // Enables Depth Testing
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(20.0, 1, 0.5, 10);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   gluLookAt(0, 0.3, 3.5,  0.0, 0.4, 0.0,  0.0, 1.0, 0); 
	 lighting();
	 
   /* When the shading model is GL_FLAT only one colour per polygon is used, 
      whereas when the shading model is set to GL_SMOOTH the colour of 
      a polygon is interpolated among the colours of its vertices.  */
   glShadeModel(GL_SMOOTH) ; 

   glutDisplayFunc (&display) ;
   glutKeyboardFunc(&keyboard);

}

// Main Program
int main(int argc, char** argv)
{
	// Initialize GLUT
	/* Initialize GLUT state - glut will take any command line arguments 
	see summary on OpenGL Summary */  
	glutInit (&argc, argv);
	//fd = open_port();
	//init_port(fd);
	ff = fopen("Data.csv","w");
	gpdata = fopen("result.dat","w");

	/* Select type of Display mode:   
		Double buffer 
		RGBA color
		Alpha components supported 
		Depth buffer 
	*/  
	//glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB );
	/* set a 400 (width) x 400 (height) window and its position */
	glutInitWindowSize(img_width,img_height);	
	glutInitWindowPosition (40, 100);

	/* Open a window */  
	window = glutCreateWindow ("Simple Window");
	
	/* Open Camera */
	
	cap.open(deviceID);
	if(!cap.isOpened())
	{
		cerr << "Error : Unable to open camera \n";
		return -1;
	}
   
	/* Initialize our window. */
	init() ;
	camera_window(); 
	init_robot();
	forward_kinematic();
	xakhir[0] = xr[0]; 
	xakhir[1] = xr[1];

	/* Register the function to do all our OpenGL drawing. */
	glutIdleFunc(&Sim_main); // fungsi untuk simulasi utama

	/* Start Event Processing Engine */ 
	glutMainLoop () ;
	return 0 ;
}           
