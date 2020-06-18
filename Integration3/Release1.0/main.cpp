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
#include <math.h>
#include <unistd.h> // Header file for sleeping.
#include <fcntl.h>			/* File control definitions */
#include <errno.h>			/* Error number definitions */
#include <termios.h>		/* POSIX terminal control definitions */
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "glfunc.hpp"
#include "improc.hpp"

using namespace cv;
using namespace std;

/* ascii code for the escape key */
#define ESCkey	27

/* The number/handle of our GLUT window */
int window, wcam;  
 
FILE *ff;
FILE *gpdata;

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
bool JS = 0; // Joint space flag
bool TS = 0; // Task space flag
bool VS = 0; // Visual-Servoing flag
int TM = 0; // Target move

// Param robot
float l[2] = {1,1};

// Param Task Space
//best so far 0.2 & 0.1
float Kp = 0.2;
float Kd = 0.1;

float Period = 3;
float step = 50 * Period;

float xr[2], xawal[2], xakhir[2],xcmd[2],xcmdold[2];
float dxcmd[2], dxcmdold[2], ddxcmd[2];
float q[2], dq[2], ddq[2];
float Jb1[2], Jb2[2], iJb1[2], iJb2[2];
float det;
float dt = 0.02;

// Param Joint Space Spesific
float KpJ = 0.02;
float KdJ = 0.005;

float qawal[2], qakhir[2], qcmd[2];
float dqold[2], dqcmd[2], ddqcmd[2];
float qcontrol[2];


void Sim_main(void); // Deklarasi lebih awal agar bisa diakses oleh fungsi sebelumnya
void display(void); // fungsi untuk menampilkan gambar robot / tampilan camera awal
void camera_result(void); // fungsi untuk menampilkan hasil olah camera

// Image Processing
VideoCapture cap;
int deviceID = 0;
int d_goal = 0;
/*
typedef struct{
	int d_l; // Detect Link
	int d_o; // Detect Object
	double deg_link; // Degree Link
	double deg_obj; // Degree Object
} dataImProc; */


// Communication
#define ROBOT_PORT	45000		// Roboard Port Number
#define PC_PORT	45001		// Cmd Port Number

static int g_rtfd = -1;

typedef struct
{
	float sudut_joint1; // Joint 1
	float sudut_joint2; // Sebenarnya gk disend
	float sudut_joint3; // Target
} sudut_st;

sudut_st data_send;

unsigned int error,etemp;
int  sock, rtsock, count, params[2], fd;
int  addrLen;
struct sockaddr_in addr;
struct timeval tv1, tv2;

//char *s = "127.0.0.1";
char *s = strdup("192.168.0.20");
unsigned short PORT_NO;
int cp_agl2cmd=0;
FILE *fdata;

//

// Task Space Control
void forward_kinematic()
{
	xr[0] = l[0]*cos(*tetha1) + l[1]*cos(*tetha1+*tetha2);
	xr[1] = l[0]*sin(*tetha1) + l[1]*sin(*tetha1+*tetha2);
}

void trajectory_init()
{	
	forward_kinematic();
	counter = 0;
	
	// For Joint Space
	for(int u=0; u<2; u++)
	{
		qawal[u] = q[u];
		dqcmd[u] = 0;
		dqold[u] = 0;
		ddqcmd[u] = 0;
	}	
	
	// For Task Space
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

void joint_space(int u)
{
	// Trajectory planning
	qcmd[u] = qawal[u] + (qakhir[u] - qawal[u]) * counter/step;
	// Derivative
	dqcmd[u] = (qcmd[u] - q[u]);
	ddqcmd[u] = (dqcmd[u] - dqold[u]);
	// Control signal
	qcontrol[u] = KpJ * dqcmd[u] + KdJ * ddqcmd[u];
	// Integral
	dq[u] += qcontrol[u];
	q[u] += dq[u];
	// Memory
	dqold[u] = dqcmd[u];
}

void visual_servoing()
{
	int i = 0;
}

int CreateTCPServerSocket(unsigned short port)
{
    int sock;                        /* socket to create */
    struct sockaddr_in echoServAddr; /* Local address */

    /* Create socket for incoming connections */
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        return -1;
      
    /* Construct local address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));   /* Zero out structure */
    echoServAddr.sin_family = AF_INET;                /* Internet address family */
    echoServAddr.sin_addr.s_addr = htonl(INADDR_ANY); /* Any incoming interface */
    echoServAddr.sin_port = htons(port);              /* Local port */

    /* Bind to the local address */
    if (bind(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0) {
        perror("bind() failed");
		return -1;
	}

    return sock;
}

int CreateTCPClientSocket(unsigned short port, char *servIP)
{
   int sock;						// Socket
   struct sockaddr_in echoServAddr; /* Local address */

    /* Create socket for incoming connections */
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        return -1;
      
    /* Construct the server address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));     /* Zero out structure */
    echoServAddr.sin_family      = AF_INET;             /* Internet address family */
    echoServAddr.sin_addr.s_addr = inet_addr(servIP);   /* Server IP address */
    echoServAddr.sin_port        = htons(port); /* Server port */

    // To Enable Broadcast message 255.255.255.255 (some network adapter cann't receive unless broadcast)
	int on = 1;
	setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *)&on, sizeof(on));

	/* Establish the connection to the echo server */
    if (connect(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0) {
        perror("connect() failed");
		return -1;
	}

    return sock;
}
	
char *EncodeIPAddress(char *s, struct sockaddr_in *sin)
{
	sprintf(s, "port :%d", 
		htons(sin->sin_port)
	);
	return s;
}

void send_data(float a, float b, float c)
{
	data_send.sudut_joint1 = a;
	data_send.sudut_joint2 = b;
	data_send.sudut_joint3 = c;
	// send 1 packet
	//printf("Fine : %.2f\n",data_send.sudut_joint1);
	int u = send(sock, &data_send, sizeof(data_send), 0);
	//printf("te %d + %d + %d + %d\n", u, sock, &data_send, sizeof(data_send));
	if(u < 0) {
		//printf("Not fine : %.2f\n",data_send.sudut_joint1);
		close(sock);
		exit(0);
	}	
}

int init_connect()
{	
	PORT_NO = ROBOT_PORT;
	if((sock = CreateTCPClientSocket(PORT_NO, s)) < 0) 
	{
		exit(-1);
	}
	printf("tx bound to address %d\n", PORT_NO);
	gettimeofday(&tv1, NULL);
}

void Sim_main(void)
{
	unsigned long Xr=0,Yr=0, Xg=0,Yg=0, Xb=0,Yb=0; // titik untuk menghitung sum
	int Nr=0, Ng=0, Nb=0;
	static unsigned int Rx,Ry, Gx,Gy, Bx,By; // untuk menyimpan hasil titik berat
	unsigned int i,j,k;
	glutSetWindow(window);
	
	
	Mat imgOriginal;
	
	dataImProc dataip;
	dataImProc *dataipptr = &dataip;

	bool bSuccess = cap.read(imgOriginal); 
	//imgOriginal = imread("SamplePict2.png", CV_LOAD_IMAGE_COLOR);	
	if (!bSuccess) 
	{
		cout << "Cannot read a frame from video stream" << endl;
	}
	else
	{	
		Improc(imgOriginal, dataipptr);
	}
	
	forward_kinematic();
	if(JS)
	{
		joint_space(0);
		if(counter == step)
		{
			JS = 0;
		}
		else if(counter < step)
		{
			counter++;
		}
		send_data(q[0]*RTD,q[1]*RTD,1.01); // send data UDP
		cout << counter << "_" << qawal[0] << "_" << q[0] << "_" << qakhir[0] << endl;
	}
	if(TS)
	{
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
			fprintf(ff,"%d, %.4f, %.4f, %.4f, %.4f, \n", counter, xcmd[0], xr[0], xcmd[1], xr[1]);
			fprintf(gpdata,"%d, %.3f, %.3f, %.3f, %.3f, \n", counter, xcmd[0], xr[0], xcmd[1], xr[1]);
			sleep(0.02);
			if(counter == step)
			{
				TS = 0;
			}
		}
	}
	if(VS)
	{
		if(counter == 0)
		{
			// Init visual servoing
			d_goal = 0;
			if(dataip.d_l == 0)
			{
				cout << "Link not detected, operation aborted" << endl;
				VS = 0;
			}
			else if(dataip.d_o == 0)
			{
				cout << "Object not detected, operation aborted" << endl;
				VS = 0;
			}
			else
			{
				q[0] = dataip.deg_link * DTR;	
				qawal[0] = q[0];
				qakhir[0] = dataip.deg_obj * DTR;
				d_goal = 1;
				cout << "Operation start : " << counter << "_" << qawal[0]*RTD << "_" << q[0]*RTD << "_" << qakhir[0]*RTD << endl;
			}
			
		}
		if(d_goal)
		{
			q[0] = dataip.deg_link * RTD;
			joint_space(0);
			send_data(q[0]*RTD,0,0);
			cout << counter << "+++" << qawal[0]*RTD << dataip.deg_link << "+++" << q[0]*RTD << "_" << qakhir[0]*RTD << endl;
		}
		
		if(counter < step)
		{
			q[0] = dataip.deg_link * DTR;
			counter++;
		}
		else if(counter == step)
		{
			printf("Pukul objek");
			send_data(q[0]*RTD,1,0); // Pukul
			VS = 0;
		}
	}
	if(TM != 0)
	{
		send_data(q[0]*RTD,0,TM);
		TM = 0;
	}
	
	//cout << dataip.deg_link << endl;
	
	*tetha1=q[0];
	*tetha2=q[1];
	
	/*
	header = 0xF5;
	send1 = *tetha1*RTD + 90;
	send2 = *tetha2*RTD + 90;
	printf("## %u ## %u ##\n",(unsigned)send1,(unsigned)send2);
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
   	  if (gambarR[j][i]>thr) {Xr+=i;Yr+=j;Nr++;
   	  	  if (debug) printf("%d,%d ",i,j);}
   	  if (gambarG[j][i]>thr) {Xg+=i;Yg+=j;Ng++;}
   	  if (gambarB[j][i]>thr) {Xb+=i;Yb+=j;Nb++;}
   }
  if (debug) {
  	printf("\n");
  	for (k=0;k<img_width*img_height;k++)
  	  if (*(&gambarR[0][0]+k)>thr) printf("%d ",k);
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
	  case 'Z': TS = 1; trajectory_init(); xakhir[0] = xr[0]+0.1; xakhir[1] = xr[1]+0.1; break;
	  case 'z': TS = 1; trajectory_init(); xakhir[0] = xr[0]-0.1; xakhir[1] = xr[1]-0.1; break;
	  case 'X': JS = 1; trajectory_init(); qakhir[0] = q[0]+(10*DTR); break;
	  case 'x': JS = 1; trajectory_init(); qakhir[0] = q[0]-(10*DTR); break;
	  case 'C': VS = 1; trajectory_init(); break;
	  case 'c': VS = 1; trajectory_init(); break;
	  case 'T': TM = 1; break; //Target movement + 10 deg
	  case 't': TM = 2; break; //- 10 deg
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
	init_connect(); // <+++++++++++ init UDP

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
	q[0] = *tetha1;
	q[1] = *tetha2;
	xakhir[0] = xr[0]; 
	xakhir[1] = xr[1];

	/* Register the function to do all our OpenGL drawing. */
	glutIdleFunc(&Sim_main); // fungsi untuk simulasi utama

	/* Start Event Processing Engine */ 
	glutMainLoop () ;
	return 0 ;
}           
