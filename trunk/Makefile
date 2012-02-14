CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=${LIB_FLAGS} -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore

all: multipoint

multipoint: multipoint.cpp
	g++ ${CFLAGS} -c multipoint.cpp
	g++ ${CFLAGS} -o multipoint multipoint.o ${CPP_LIB_FLAGS} ${LIB_LINK}

clean:
	rm -rf *.o
	rm -rf multipoint