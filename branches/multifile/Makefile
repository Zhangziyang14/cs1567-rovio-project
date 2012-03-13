CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=${LIB_FLAGS} -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore

rovio: Fir.o PID.o Kalman.o driver.o
	g++ ${CFLAGS} -o rovio Fir.o PID.o Kalman.o driver.o ${CPP_LIB_FLAGS} ${LIB_LINK}

Fir.o: Fir.cpp Fir.h
	g++ ${CFLAGS} -c Fir.cpp

PID.o: PID.cpp PID.h
	g++ ${CFLAGS} -c PID.cpp

Kalman.o: Kalman.cpp Kalman.h
	g++ ${CFLAGS} -c Kalman.cpp

driver.o: driver.cpp Functions.h
	g++ ${CFLAGS} -c driver.cpp

clean:
	rm -rf *.o
	rm -rf rovio
	rm -f *.o TR.csv