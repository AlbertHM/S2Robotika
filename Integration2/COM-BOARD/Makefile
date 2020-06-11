
CCGL = gcc
FLAGS = 
LIBS = -I../../include -L../../lib -lRBIO -lstdc++

all:	roboardSide
roboardSide: roboardSide.cpp
	$(CCGL) $(FLAGS) $^ $(LIBS) -o $@
clean:
	rm -rf *.exe
