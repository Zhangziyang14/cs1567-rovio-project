CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=${LIB_FLAGS} -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore
PROGRAM=Test

all: ${PROGRAM}

${PROGRAM}: ${PROGRAM}.cpp
	g++ ${CFLAGS} -c ${PROGRAM}.cpp
	g++ ${CFLAGS} -c Fir.cpp
	g++ ${CFLAGS} -c PID.cpp
	g++ ${CFLAGS} -c Kalman.cpp
	g++ ${CFLAGS} -c Robot.cpp
	g++ ${CFLAGS} -o ${PROGRAM} ${PROGRAM}.o Fir.o PID.o Kalman.o Robot.o ${CPP_LIB_FLAGS} ${LIB_LINK}

clean:
	rm -rf *.o
	rm -rf robot_test_example++