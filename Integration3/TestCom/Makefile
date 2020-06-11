
CCGL = gcc
FLAGS = 
#LIBS = -I../../include -L../../lib -lRBIO -lstdc++
LIBS = -I../../include -L../../lib -lstdc++
#LIBS = 

all: roboardSide comPC
NR: netroboard.c
	$(CCGL) $(FLAGS) $^ $(LIBS) -o $@

roboardSide: roboardSide.c
	$(CCGL) $(FLAGS) $^ $(LIBS) -o $@

comPC: comPC.c
	$(CCGL) $(FLAGS) $^ $(LIBS) -o $@
	
clean:
	rm -rf comPC roboardSide *.exe
