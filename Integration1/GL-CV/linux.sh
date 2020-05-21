#!/bin/sh

# if [ $# -gt 0 ] ; then
#     base=`basename $1`
#     echo "compiling $base"
#     g++ -ggdb `pkg-config opencv --cflags --libs` $1 -o $base 
# if [ $# -gt 0 ] ; then
# 	base=`basename $1 .c`
# 	echo "compiling $base"
# 	gcc -ggdb `pkg-config opencv --cflags --libs` $base.c -o $base 

#export LD_LIBRARY_PATH=/lib:/usr/lib:/usr/local/lib #not working too -_-

#for f in /usr/local/lib/libopencv*.so.3.1; do sudo ln -s "${f##*/}" "${f%.3.1*}"; done #just once and its enough

if [ $# -gt 0 ] ; then
    base=`basename $1 .cpp`
    echo "compiling $base"
    g++ -ggdb `pkg-config opencv --cflags --libs` $base.cpp -o $base 
else
	for i in *.c; do
	    echo "compiling $i"
	    g++ -ggdb `pkg-config --cflags opencv` -o `basename $i .c` $i `pkg-config --libs opencv` -lm;
	done
	for i in *.cpp; do
	    echo "compiling $i"
	    g++ -ggdb `pkg-config --cflags opencv` -o `basename $i .cpp` $i `pkg-config --libs opencv` -lm;
	done
fi
