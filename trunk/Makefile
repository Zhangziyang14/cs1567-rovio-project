CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=${LIB_FLAGS} -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore

all: driver

driver: driver.cpp
	g++ ${CFLAGS} -c driver.cpp
	g++ ${CFLAGS} -o rovio driver.o ${CPP_LIB_FLAGS} ${LIB_LINK}

clean:
	rm -rf *.o
	rm -rf rovio