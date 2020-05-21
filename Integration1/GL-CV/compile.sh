#openGL and OpenCV
g++ -ggdb `pkg-config --cflags --libs opencv` planargl.c -o main `pkg-config --libs opencv` -lm -lGL -lGLU -lglut 
