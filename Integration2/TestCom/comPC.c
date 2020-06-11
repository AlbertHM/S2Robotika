/* Part of Integration 2
*
* comPC.hpp
* Function : Send struct of integer contain desired angle
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

//#include <arpa/inet.h> // uncomment this line if u use *.cpp

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

long i;
unsigned int error,etemp;
int  sock, rtsock, count, params[2], fd;
int  addrLen;
struct sockaddr_in addr;
struct timeval tv1, tv2;
char s1[1024], s2[200];

char *s = "127.0.0.1";
unsigned short PORT_NO;
int cp_agl2cmd=0;
FILE *fdata;

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
	//sudut_st data_send;
	data_send.sudut_joint1 = a;
	data_send.sudut_joint2 = b;
	data_send.sudut_joint3 = c;
	// send 1 packet
	printf("a1\n");
	printf("Fine : %.2f\n",data_send.sudut_joint1);
	int u = send(sock, &data_send, sizeof(data_send), 0);
	printf("te %d + %d + %d + %d\n", u, sock, &data_send, sizeof(data_send));
	if(u < 0) {
		printf("Not fine : %.2f\n",data_send.sudut_joint1);
		close(sock);
		exit(0);
	}	
}

int init_connect()
{	
	printf("ok1");
	PORT_NO = ROBOT_PORT;
	
	printf("ok2");
	if((sock = CreateTCPClientSocket(PORT_NO, s)) < 0) 
	{
		exit(-1);
		printf("error");
	}	
	printf("ok3");
	printf("tx bound to address %d\n", PORT_NO);
	gettimeofday(&tv1, NULL);
}

int main()
{
	int counter = 0;
	printf("ok0");
	init_connect();
	//send_data(10.1, 10.2, 10.3);
	for(;;) {
		send_data(10.1, 10.2, 10.3);
		usleep(1000);
		printf("Counter : %d",counter);
		counter++;
	}
	return 0;
}
