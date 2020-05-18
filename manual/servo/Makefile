servo.exe : servo.o
	gcc -o servo.exe setinit.o -L../../Lib -lRBIO -lstdc++
servo.o : setinit.cpp
	gcc -c setinit.cpp -I../../Include

.PHONY : clean
clean :
	-rm servo.o servo.exe
