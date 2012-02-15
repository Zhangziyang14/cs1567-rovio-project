CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=${LIB_FLAGS} -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore

rovio: driver.o rovioKalmanFilter.o
	g++ ${CFLAGS} -o rovio driver.o rovioKalmanFilter.o ${CPP_LIB_FLAGS} ${LIB_LINK}

driver.o: driver.cpp
	g++ ${CFLAGS} -c driver.cpp
	
rovioKalmanFilter.o: kalman/rovioKalmanFilter.c kalman/kalmanFilterDef.h
	gcc ${CFLAGS} -c kalman/rovioKalmanFilter.c

clean:
	rm -rf *.o
	rm -rf rovio
	rm -f *.o TR.csv