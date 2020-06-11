
CCGL = gcc
FLAGS = 
#LIBS = -I../../include -L../../lib -lRBIO -lstdc++
LIBS = -I../../include -L../../lib -lstdc++
#LIBS = 

all: NR roboardSide 
NR: netroboard.c
	$(CCGL) $(FLAGS) $^ $(LIBS) -o $@

roboardSide: roboardSide.cpp
	$(CCGL) $(FLAGS) $^ $(LIBS) -o $@

#comPC : comPC.hpp
#	$(CCGL) $(FLAGS) $^ $(LIBS) -o $@
	
clean:
	rm -rf NR roboardSide *.exe
