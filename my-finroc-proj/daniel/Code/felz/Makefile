INCDIR = -I.
DBG    = -g
OPT    = -O3
CPP    = g++
CFLAGS = $(OPT) $(INCDIR)
#CFLAGS = $(DBG) $(OPT) $(INCDIR)
LINK   = -lm 

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

all: segment

segment: segment.cpp segment-image.h segment-graph.h disjoint-set.h
# segment.cpp segment-image_contours.h segment-graph.h disjoint-set.h
	
	$(CPP) $(CFLAGS) -o segment segment.cpp $(LINK)

clean:
	/bin/rm -f segment *.o

clean-all: clean
	/bin/rm -f *~ 



