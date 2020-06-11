/* Part of Integration 1
*
* roboardSide.cpp
* Function : Receive struct of joint angle and control roboard
*
* Albert H.M., S.T.
*
*/

// Note : Jangan simpan dalam .cpp, inet_addr tidak akan berhasil dicompile. Simpan dalam *.c

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

#define ROBOT_PORT	45000		// Roboard Port Number
#define PC_PORT	45001		// Cmd Port Number

static int g_rtfd = -1;

typedef struct
{
	float sudut_joint1; // Joint 1
	float sudut_joint2; // Sebenarnya gk disend
	float sudut_joint3; // Target
} sudut_st;

sudut_st data_recv;

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

int main(int argc, char *argv[])
{

	PORT_NO=ROBOT_PORT;
	if((sock = CreateTCPServerSocket(PORT_NO)) < 0) 
	{
		exit(-1);
	}
	addrLen = sizeof(addr);
	if(getsockname(sock, &addr, &addrLen) < 0 ) 
	{
		close(sock);
		return -1;
	}
	printf("rx bound to address %s\n", EncodeIPAddress(s1, &addr));
	count = 0;
	error=0;
	gettimeofday(&tv1, NULL);
	for(;;) {
		count++;
		if (recv(sock, &data_recv, sizeof(data_recv), 0)!=sizeof(data_recv)) 
		{
			printf("Receive data not equal\n");
		} 
		else 
		{
			printf("Receive data S1:%f, S2:%f, S3:%f\n", data_recv.sudut_joint1, data_recv.sudut_joint2, data_recv.sudut_joint3);
		}
		usleep(1000);
	}
	fclose(fdata);
	return 0;
}
