#openGL and OpenCV
g++ -ggdb `pkg-config --cflags --libs opencv` main.cpp -o main `pkg-config --libs opencv` -lm -lGL -lGLU -lglut 
