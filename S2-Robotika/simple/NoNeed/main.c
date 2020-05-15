/**
 * Display video from webcam and track user's eye with
 * manually selected template.
 *
 * Author    Nash <me [at] nashruddin.com>
 * License   GPL
 * Website   http://opencv-code.info
 */

#include <stdio.h>
#include "cv.h"
#include "highgui.h"

#define  TPL_WIDTH       12      /* template width       */
#define  TPL_HEIGHT      12      /* template height      */
#define  WINDOW_WIDTH    24      /* search window width  */
#define  WINDOW_HEIGHT   24      /* search window height */
#define  THRESHOLD       0.3

#define max3(r,g,b) ((r)>(g)?((r)>(b)?(r):(b)):((g)>(b)?(g):(b)))
#define min3(r,g,b) ((r)<(g)?((r)<(b)?(r):(b)):((g)<(b)?(g):(b)))


IplImage *frame, *tpl, *tm;
int      object_x0, object_y0, is_tracking = 0;

void mouseHandler( int event, int x, int y, int flags, void *param );
void trackObject();

/*
#include <math.h>
static inline int cvRound(float value)
{
    return (int)roundf(value);
}*/


/* main code */
int main( int argc, char** argv )
{
    CvCapture   *capture;
    int         key;
    int i, j, r, g, b, byte;
    unsigned long maxred;
    
    /* initialize camera */
    capture = cvCaptureFromCAM( 0 );

    /* always check */
    if( !capture ) return 1;
    printf("masuk");

    /* get video properties, needed by template image */
    frame = cvQueryFrame( capture );
    if ( !frame ) return 1;
    	
    IplImage *convert=cvCreateImage( cvGetSize(frame), 8, 3 );
    cvCvtColor(frame,convert,CV_BGR2HSV); // convert from BGR to HSV
    
    /* create template image */
    tpl = cvCreateImage( cvSize( TPL_WIDTH, TPL_HEIGHT ), 
                         frame->depth, frame->nChannels );
    
    /* create image for template matching result */
    tm = cvCreateImage( cvSize( WINDOW_WIDTH  - TPL_WIDTH  + 1,
                                WINDOW_HEIGHT - TPL_HEIGHT + 1 ),
                        IPL_DEPTH_32F, 1 );
    
    /* create a window and install mouse handler */
    cvNamedWindow( "video", CV_WINDOW_AUTOSIZE );
    cvSetMouseCallback( "video", mouseHandler, NULL );
    
    
     frame = cvQueryFrame( capture );
int width     = frame->width;
int height    = frame->height;
int nchannels = frame->nChannels;
int step      = frame->widthStep;
    printf("w:%d h:%d n:%d s:%d\n",width,height,nchannels,step);
    while( key != 'q' ) {
        /* get a frame */
        frame = cvQueryFrame( capture );

        /* always check */
        if( !frame ) break;

        /* 'fix' frame */
        //cvFlip( frame, frame, -1 );
        frame->origin = 0;
        
         maxred=0;
 
  
 
         /* setup the pointer to access image data */
        uchar *data = ( uchar* )frame->imageData;    
        int x, v, s, h, h2, f, In;
         
        /* convert to grayscale manually */
        
        for( i = 0 ; i < height ; i++ ) {
            for( j = 0 ; j < width ; j++ ) {
                b = data[i*step + j*nchannels + 0];
                g = data[i*step + j*nchannels + 1];
                r = data[i*step + j*nchannels + 2];
                
                //byte = ( r + g + b ) / 3;
                       
                /*data[i*step + j*nchannels + 0] = byte;
                data[i*step + j*nchannels + 1] = byte;
                data[i*step + j*nchannels + 2] = byte;
                */
                /* convert RGB to hsv */
                x = min3(r,g,b);
                v = max3(r,g,b);
                if (v==x) { h=0; s=0; }
                else {
        								s = (float)(v-x)/(float)v * 255.0;
        								
        								/*if (r==v) h2= (float)(g-b) / (float)(v-x); // antara warna kuning & magenta
        								else if(g==v) h2= 2+(float)(b-r) / (float)(v-x); // antara warna cyan dan kuning
        								else h2= 4+(float)(r-g) / (float)(v-x); // antara magenta dan cyan
        							  h2 *=60;  // convert to degree
        							  if (h2<0) h2+=360;
        								*/	
        								
                        f = ( r==v ) ? (g-b) : (( g==v ) ? ( b-r ) : ( r-g ));
                        In = ( r==v ) ? 0 : (( g==v ) ? 2 : 4 );
                        h = ( In + (float)f/(float)(v-x) )/6.0*255.0;
                        if ( h < 0 ) h += 255;
                        if ( h < 0 || h > 255 || v < 0 || v > 255 ) {
                            printf( "%d %d: bad HS values: %d\n",
                                     h2,h, s );
                            exit( 0 );
                        }
                	
                }
                if ((v>253) && (s>10)) printf("r:%d g:%d b:%d - (%d,%d) h:%d s:%d \n",r,g,b,j,i,h,s);
            }
        }
                
        /* perform tracking if template is available */
        if( is_tracking ) trackObject();
        
        /* display frame */
        cvShowImage( "video", frame );
        
        /* exit if user press 'q' */
        key = cvWaitKey( 1 );
    }

    /* free memory */
    cvDestroyWindow( "video" );
    cvReleaseCapture( &capture );
    cvReleaseImage( &tpl );
    cvReleaseImage( &tm );
    
    return 0;
}

/* mouse handler */
void mouseHandler( int event, int x, int y, int flags, void *param )
{
	/* user clicked the image, save subimage as template */
    if( event == CV_EVENT_LBUTTONUP ) {
        object_x0 = x - ( TPL_WIDTH  / 2 );
        object_y0 = y - ( TPL_HEIGHT / 2 ); 
        
		cvSetImageROI( frame, 
                       cvRect( object_x0, 
                               object_y0, 
                               TPL_WIDTH, 
                               TPL_HEIGHT ) );
        cvCopy( frame, tpl, NULL );
        cvResetImageROI( frame );

        /* template is available, start tracking! */
        fprintf( stdout, "Template selected. Start tracking... \n" );
        is_tracking = 1;
    }
}

/* track object */
void trackObject()
{
    CvPoint minloc, maxloc;
    double  minval, maxval;

    /* setup position of search window */
    int win_x0 = object_x0 - ( ( WINDOW_WIDTH  - TPL_WIDTH  ) / 2 );
    int win_y0 = object_y0 - ( ( WINDOW_HEIGHT - TPL_HEIGHT ) / 2 );
    
	/*
	 * Ooops, some bugs here.
	 * If the search window exceed the frame boundaries,
	 * it will trigger errors.
	 *
	 * Add some code to make sure that the search window 
	 * is still within the frame.
	 */
	
    /* search object in search window */
    cvSetImageROI( frame, 
                   cvRect( win_x0, 
                           win_y0, 
                           WINDOW_WIDTH, 
                           WINDOW_HEIGHT ) );
    cvMatchTemplate( frame, tpl, tm, CV_TM_SQDIFF_NORMED );
    cvMinMaxLoc( tm, &minval, &maxval, &minloc, &maxloc, 0 );
    cvResetImageROI( frame );
    
    /* if object found... */
    if( minval <= THRESHOLD ) {
        /* save object's current location */
        object_x0 = win_x0 + minloc.x;
        object_y0 = win_y0 + minloc.y;

        /* and draw a box there */
        cvRectangle( frame,
                     cvPoint( object_x0, object_y0 ),
                     cvPoint( object_x0 + TPL_WIDTH, 
					          object_y0 + TPL_HEIGHT ),
                     cvScalar( 0, 0, 255, 0 ), 1, 0, 0 );
    } else {
        /* if not found... */
        fprintf( stdout, "Lost object.\n" );
        is_tracking = 0;
    }
}
