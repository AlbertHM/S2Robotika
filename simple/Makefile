# for Opengl

CCGL=g++
GL_CFLAGS = -O2 -DSHM -DHZ=100
XLIBS = -lXext -lXmu -lXi -lX11
GL_LIBS =  
#GL_LIBS =  -I/usr/X11R6/include -L/usr/lib -L/usr/X11R6/lib  #gak jalan di opengl 2.x 
#CYG_OpenGL = -I/usr/include/opengl -I/cygdrive/c/OpenCV/include/opencv -L /cygdrive/c/OpenCV/lib -lglut32 -lglu32 -lopengl32 #-lcygipc
CYG_OpenGL = -I/usr/include/opengl -I/cygdrive/c/OpenCV/include/opencv -L /cygdrive/c/OpenCV/lib -lglut32 -lglu32 -lopengl32 #-lcygipc
LINUX_OpenGL =  -lglut -lGLU -lGL $(XLIBS) 
GL_LIBS += $(CYG_OpenGL) -lm -llibcv200 -llibcvaux200 -llibcxcore200 -llibhighgui200 

INC_FILES= $(wildcard *.cpp) $(wildcard *.o)


INC_FILES= $(wildcard *.cpp) $(wildcard *.o)

all: deteksi

deteksi: main.c 
	$(CCGL) $(GL_CFLAGS) $^ $(GL_LIBS) -o $@ 



clean:
	rm -rf *.exe