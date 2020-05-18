/* Part of Integration 1
*
* Receiver.c
* Function : Receive struct of joint angle and control roboard
*
* Albert H.M., S.T.
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <roboard.h>

#define RECEIVER		1
#define TRANSCEIVER		2
//#define PC 1


#define ROBOT_PORT	45000		// Roboard Port Number
#define PC_PORT	45001		// Cmd Port Number

static int g_rtfd = -1;

// Structure
typedef struct	{
	float X_titik1;
	float X_titik2;
	float X_titik3;
} pc_t;

typedef struct  {
	float sudut_sendi1;
	float sudut_sendi2;
} roboard_t;

typdef struct pc_t DataSudut;

#ifdef PC
  pc_t		data_send;
  roboard_t	data_recv;
  #define CONTROL_SIDE	1
#else
  roboard_t		data_send;
  pc_t			data_recv;
  #define CONTROL_SIDE	0
#endif

long Convert2MS(int x)
{
	/*
	Function : Converting sudut in degree into PWM length signal
	
	Return : PWM length signal
	*/
	int m = 0L;
	int c = 0L;
	return m*(long)x+c
}

void MoveRobot(DataSudut ds)
{
	/*
	Function : assigning new target of joint degree and move it in 20ms
	
	Return : None
	*/
	frame[2] = Convert2MS(ds.sudut_sendi1);
	frame[3] = Convert2MS(ds.sudut_sendi2);
	rcservo_MoveTo(frame, 20L) // move in 20 ms
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
		
char *
EncodeIPAddress(char *s, struct sockaddr_in *sin)
{
	sprintf(s, "port :%d", 
		htons(sin->sin_port)
	);
	return s;
}


int
main(int argc, char *argv[])
{
	// =========== INIT Roboard =========== //
	
	unsigned long home[32]  = {0L};
    unsigned long frame[32] = {0L};
    
    // First set the correct RoBoard version
    roboio_SetRBVer(RB_100);
    //roboio_SetRBVer(RB_100RD);  // if your RoBoard is RB-100RD
    //roboio_SetRBVer(RB_110);    // if your RoBoard is RB-110
    //roboio_SetRBVer(RB_050);    // if your RoBoard is RB-050

    rcservo_SetServo(RCSERVO_PINS3, RCSERVO_SERVO_DEFAULT_NOFB);     // select the servo model on pin S1 as non-feedback servo
    rcservo_SetServo(RCSERVO_PINS4, RCSERVO_SERVO_DEFAULT_NOFB);     // select the servo model on pin S2 as non-feedback servo
    if (rcservo_Init(RCSERVO_USEPINS3 + RCSERVO_USEPINS4) == false)  // set PWM/GPIO pins S1 & S2 as Servo mode
    {
        printf("ERROR: fail to init RC Servo lib (%s)!\n", roboio_GetErrMsg());
        return -1;
    }

    home[2] = home[3] = 1500L;  // set the initial home position of all servos as 1500us
    rcservo_EnterPlayMode_HOME(home);  // enter Action Playing Mode for moving servos
	
	// =========== INIT Roboard =========== /
	
	long i;
	unsigned int error,etemp;
	int  sock, rtsock, count, params[2], fd;
	int  addrLen;
	struct sockaddr_in addr;
	struct timeval tv1, tv2;
	char *s, s1[1024], s2[200];
	unsigned short PORT_NO;
	int cp_agl2cmd=0;
	FILE *fdata;

	if (CONTROL_SIDE) 
		printf("Control side\n");
	else
		printf("Robot side\n");

	if(argc < 2) {
		fprintf(stderr, "usage: netpc [ru|tu] [<address>]\n");
		exit(-1);
	}
	s = (argc < 3)? "" : argv[2];

	if(strcmp(argv[1], "ru") == 0) {
		if (CONTROL_SIDE) 
			PORT_NO=PC_PORT;
		else 
			PORT_NO=ROBOT_PORT;
		if((sock = CreateTCPServerSocket(PORT_NO)) < 0) {
			exit(-1);
		}
		addrLen = sizeof(addr);
		if(getsockname(sock, &addr, &addrLen) < 0 ) {
		    	close(sock);
		    	return -1;
		}
		printf("rx bound to address %s\n", EncodeIPAddress(s1, &addr));
		count = 0;
		error=0;
		gettimeofday(&tv1, NULL);
		for(;;) {
			count++;
			if (recv(sock, &data_recv, sizeof(data_recv), 0)!=sizeof(data_recv)) {
				printf("Receive data not equal\n");
			} else {
				#ifdef PC
					printf("Receive data q1:%f, q2:%f\n", data_recv.sudut_sendi1, data_recv.sudut_sendi2);
					MoveRobot(DataSudut data_recv) // <================ Moving Servo
				#else
					printf("Receive data t1:%f, t2:%f, t3:%f\n", data_recv.X_titik1, data_recv.X_titik2, data_recv.X_titik3);
				#endif
			}
			usleep(1000000);
		}
		fclose(fdata);
	} else if(strcmp(argv[1], "tu") == 0) {
		if (CONTROL_SIDE) {
			PORT_NO=ROBOT_PORT; 
		} else {
			PORT_NO=PC_PORT;	
		}
		if((sock = CreateTCPClientSocket(PORT_NO, s)) < 0) {
			exit(-1);
		}
		
		printf("tx bound to address %d\n", PORT_NO);

		gettimeofday(&tv1, NULL);
#ifdef PC
		data_send.X_titik1=0.5;
		data_send.X_titik2=0.8;
		data_send.X_titik1=0.9;
#else
		data_send.sudut_sendi1=1.2;
		data_send.sudut_sendi2=1.5;
#endif
		for(;;) {
			usleep(1000000);

			// send 1 packet
			if(send(sock, &data_send, sizeof(data_send), 0) < 0) {
			    close(sock);
			    exit(0);
			}
		}
	} else {
		fprintf(stderr, "usage: netpc [ru|tu] [<address>]\n");
		exit(-1);
	}
	return 0;
}
