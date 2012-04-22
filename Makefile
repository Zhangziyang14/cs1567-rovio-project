CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
CPP_LIB_FLAGS=${LIB_FLAGS} -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore
PROGRAM=Test
DEPS=${PROGRAM}.o Fir.o FirTheta.o PID.o Kalman.o Robot.o Camera.o

all: ${PROGRAM}

debug: CFLAGS +=-DDEBUG -ggdb -Wall -g
debug: ${PROGRAM}

${PROGRAM}: ${DEPS}
	g++ ${CFLAGS} -o ${PROGRAM} ${DEPS} ${CPP_LIB_FLAGS} ${LIB_LINK}

%.o: %.cpp
	g++ ${CFLAGS} -c $<

clean:
	rm -rf *.o
	rm -rf *.jpg
	rm -rf *.txt
	rm -rf ${PROGRAM}
	rm -rf core.*
