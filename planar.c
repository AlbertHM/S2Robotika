#include <math.h>
//#include "serial.h"
#define PI		3.14159265358
#define DTR 	PI/180.0				   // Conversi degree to radian
#define RTD 	180.0/PI				   // Conversi degree to radian

#define L1	0.3   // link1
#define L2	0.2   // link2

#define KIRI 1
#define KANAN 2

float q1;
float q2;
float objx=0.3;
float objy=0.5;
int mode = KIRI;

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
/*
void Retrieve_serial(void) {
  int retval=1, i,j,k,l;

  unsigned char sdata[4]; 
  unsigned char baca;
  
  
	i=1;

  while (i>0) {
    fcntl(fd, F_SETFL, FNDELAY); 
    i=read(fd, &baca, 1);
    if ((i==1) && (baca == 0xF5)) {
    	//printf("masuk\n");
    	sdata[0]=baca;
    		while (i<3) {
    		  if (read(fd, &baca, 1)>0) {sdata[i]=baca; i++;}
    		}
   	   //printf("terbaca %x  %x  %x \n",sdata[0],sdata[1],sdata[2]);
   	   q1=(sdata[1])*180.0/255.0*DTR;
   	   q2=((sdata[2])*360.0/255.0-180.0)   *DTR;
    }
  } 

}


void hitung_robot() { 
	Retrieve_serial();
	
}*/
