#include <math.h>
#include <stdio.h>
#include <GL/glut.h> // Header File For The GLUT Library
#include <GL/gl.h> // Header File For The OpenGL32 Library
#include <GL/glu.h> // Header File For The GLu32 Library

#define PI		3.14159265358
#define DTR 	PI/180.0				   // Conversi degree to radian
#define RTD 	180.0/PI				   // Conversi degree to radian

#define L1	0.3   // link1
#define L2	0.2   // link2

#define KIRI 1
#define KANAN 2

// ROBOT MODEL PARAMATER
#define Xoffset	0.0	
#define Yoffset	0.0
#define Zoffset	0.2
#define img_height 200
#define img_width 350
#define thr 230

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


/* To draw a quadric model */
GLUquadricObj *obj;

float q1;
float q2;
float objx=0.3;
float objy=0.5;
int mode = KIRI;

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

void init_robot()
{
	q1=10.0 * DTR;
	q2=30.0 * DTR;
}


void fncgerak(int count)
{
	if(mode==KIRI)
	{
		q1+=2*DTR;
		q2+=1*DTR;
	}
	else
	{
		q1-=2*DTR;
		q2-=1*DTR;
	}
	if(count%40 == 0)
	{
		if(mode == KIRI)
		{
			mode = KANAN;
		}
		else
		{
			mode = KIRI;
		}
	}
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
    glRotatef(q1*RTD,0,0,1);
    glPushMatrix();
      // Gambar link1
      glRotatef(-90,1,0,0);
      glTranslatef(0,0,L1/2);
      model_cylinder(obj, 0.03, 0.03, L1, 2, pink6, yellow2);
    glPopMatrix();
    // Menuju joint-2
    glTranslatef(0,L1, 0);
    glRotatef(q2*RTD,0,0,1);
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
      glTranslatef(0,0,L2/2);
      model_cylinder(obj, 0.03, 0.03, L2, 2, yellow5, yellow2);
    glPopMatrix();
    glPushMatrix();
      glDisable(GL_LIGHTING);
      glColor3f(0.0,0.0,1.0);  // buat tanda biru (R G B) = 0 0 1             
      glTranslatef(0,L2,0.04); // pindahkan offset 1cm diatas sendi 2
      gluDisk(obj, 0.001, 0.01, 20, 2); // bikin tanda lingkaran merah diatas sendi 2 
      glEnable(GL_LIGHTING);
    glPopMatrix();
  glPopMatrix();
  glPushMatrix();
    glTranslatef(objx, objy, Zoffset/2);
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
