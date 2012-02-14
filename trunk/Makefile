CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=${LIB_FLAGS} -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore

all: driver

driver: driver.cpp rovioKalmanFilter.o
	g++ ${CFLAGS} -c driver.cpp
	g++ ${CFLAGS} -o rovio driver.o rovioKalmanFilter.o ${CPP_LIB_FLAGS} ${LIB_LINK}
	
rovioKalmanFilter.o: kalman/rovioKalmanFilter.c
	gcc ${CFLAGS} -c kalman/rovioKalmanFilter.c

clean:
	rm -rf *.o
	rm -rf rovio
	rm -f *.o TR.csv