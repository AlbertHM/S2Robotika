
INCLUDE = -I/usr/include
#LIBDIR  = -L/usr/X11R6/lib 

#COMPILERFLAGS = -Wall		# display all warning
CC = gcc
CFLAGS = $(COMPILERFLAGS) $(INCLUDE) 
INC_FILES= $(wildcard *.c) $(wildcard *.o)

LIBRARIES = #-lcygipc

all:	netpc netrobot
netpc: netmaster.c
	$(CC) $(CFLAGS) -o $@ $(LIBDIR) $^ $(LIBRARIES) 
netrobot: netroboard.c
	$(CC) $(CFLAGS) -o $@ $(LIBDIR) $^ $(LIBRARIES) 
clean:
	rm -f  robot control stopcmd webcmd *.o *.exe
