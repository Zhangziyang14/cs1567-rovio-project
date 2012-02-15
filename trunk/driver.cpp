#include <robot_if++.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>

#include <stdlib.h>
#include <signal.h>
#include <fstream>

#include "fir.h"
extern "C" {
	#include "kalman/kalmanFilterDef.h"
}
//#define FILEDUMP
//#define DUMP_NS

/******define some utility macros******/
#define ABS(x) ((x)>0?(x):-(x))
#define TORADIAN(x) ((float)(x*0.0174533))//turn degree to radian
#define MINABS(x,y) (ABS(x)<ABS(y)?(x):(y))//return the one with lower absolute value
#define TODEGREE(x) ((float)((x)*57.29578)) //turn radian into degree

#define PI 3.14159265

#define ANGLE_WHEEL_RIGHT 0.523598776
#define ANGLE_WHEEL_LEFT 2.61799388
#define ANGLE_WHEEL_REAR 1.57079633

#define ACTION_TURN 0x101/*action flag pass to update function*/
#define ACTION_MOVE 0x102/*action flag pass to update function*/

#define TICKS_PER_INCH 5.5

#define OK 0
#define FAIL -1

#define FAILED(x) ((x) == OK) ? false : true

float xNSOrigin, yNSOrigin;
int xEncCoord, yEncCoord;
float yTotal;
float thetaCorrection;
int currentRoom;
float aveWeightedDegree;

float originDegree;

filter *xFilter = NULL;
filter *yFilter = NULL;
filter *rightWheelFilter = NULL;
filter *leftWheelFilter = NULL;
filter *rearWheelFilter = NULL;



kalmanFilter *kf = NULL;

RobotInterface *robot;

#ifdef FILEDUMP
std::ofstream rawDataFile;
std::ofstream filterDataFile;
#endif

// corrects theta to 0 at beginning of run
float CorrectTheta( float oldTheta )
{
	// positive theta origin case
	if ( thetaCorrection >= 0 )
	{
		if ( oldTheta - thetaCorrection < -PI )
			return -1 * ( oldTheta - thetaCorrection + PI );
		else
			return oldTheta - thetaCorrection;
	}
	
	// negative theta origin case
	else
	{
		if ( oldTheta + thetaCorrection > PI )
			return -1 * ( oldTheta + thetaCorrection - PI );
		else
			return oldTheta + thetaCorrection;
	}
}

// returns average of computed left and right encoder x-axis motion
float WheelAverageX( float rightEncoder, float leftEncoder )
{
	float rightFinal, leftFinal;
	rightFinal = rightEncoder * cos( ANGLE_WHEEL_RIGHT );
	leftFinal = leftEncoder * cos( ANGLE_WHEEL_LEFT );
	//printf( "WheelAverageX():\nrightFinal: %.3f leftFinal: %.3f\n", rightFinal, leftFinal );
	return (rightFinal + leftFinal) / 2;
}

// returns average of computed left and right encoder y-axis motion
float WheelAverageY( float rightEncoder, float leftEncoder )
{
	float rightFinal, leftFinal;
	rightFinal = rightEncoder * sin( ANGLE_WHEEL_RIGHT );
	leftFinal = leftEncoder * sin( ANGLE_WHEEL_LEFT );
	//printf( "WheelAverageY():\nrightFinal: %.3f leftFinal: %.3f\n", rightFinal, leftFinal );
	return (rightFinal + leftFinal) / 2;
}

float NSDegreeToMyDegree(float oldD, int roomID){
	float degree = oldD;
	switch(roomID) {
		case 2: //room 2
			degree = 360+75 - oldD;
			break;
		case 3: //room3
			degree = 360 - oldD;
			break;
		case 4: //room 4
			degree = 360+55 - oldD;
			break;
		case 5: //room 5
			degree = 360+1 - oldD;
			break;
		default:
			//printf("room %d, Man, are you sure you are not in outer space!\n",roomID);
			return 0;
	}
	return (float)(((int)degree)%360);
}

void updateNorthStar(int flag,int roomID){
	int step = 0;
	float degree;
	float rawD;

	if(flag==ACTION_TURN){
		step = 0;
		degree = 0;
		rawD = 0;
		do{
			while(robot->update() != RI_RESP_SUCCESS) {
				printf("Failed to update sensor information!\n");
			}
			if(step>13){
				degree += NSDegreeToMyDegree(TODEGREE(robot->Theta()), robot->RoomID());
				rawD += TODEGREE(robot->Theta());
			}
			step++;
			
		}
		while(step<20);

		aveWeightedDegree = degree/6;
	}

	//printf("updataNS: degree: %4.2f raw:%4.2f\n",aveWeightedDegree,rawD/10);

}

	
// initialize filters and prime coord filters w/ 3 readings
int InitializeFirFilters( RobotInterface *robot )
{
	if ( robot->update() == RI_RESP_SUCCESS ) {		
		// initialize filters
		xFilter = FirFilterCreate();
		yFilter = FirFilterCreate();
		rightWheelFilter = FirFilterCreate();
		leftWheelFilter = FirFilterCreate();
		rearWheelFilter = FirFilterCreate();
		
		// prime filters w/ 3 readings
		for ( int i=0; i<3; i++ )
		{
			robot->update();
			FirFilter( xFilter, robot->X() );
			FirFilter( yFilter, robot->Y() );
		}
		
		return OK;
	}
	
	return FAIL;
}
// initializes kalman filter to default values
int InitializeKalmanFilter( RobotInterface *robot )
{
	float *initPose = (float *)malloc(sizeof(float) * 3);
	float *velocity = (float *)malloc(sizeof(float) * 3);
	int deltat = 1;
	
	kf = (kalmanFilter *)malloc(sizeof(kalmanFilter));

	if ( robot->update() == RI_RESP_SUCCESS )
	{
		initPose[0] = FirFilter( xFilter, robot->X() );
		initPose[1] = FirFilter( yFilter, robot->Y() );
		initPose[2] = CorrectTheta( robot->Theta() );
		
		velocity[0] = 0.0;
		velocity[1] = 0.0;
		velocity[2] = 0.0;
		
		initKalmanFilter( kf, initPose, velocity, deltat );
		
		return OK;
	}
	
	return FAIL;
}

// updates kalman filter with current sensor data, returns prediction array
float *UpdateKalman()
{
	float *curPose = (float *)malloc(sizeof(float) * 3);
	float *curWheelEnc = (float *)malloc(sizeof(float) * 3);
	float *predicted = (float *)malloc(sizeof(float) * 9);
	
	float x, y, theta;
	
	curPose[0] = FirFilter( xFilter, robot->X() );
	curPose[1] = FirFilter( yFilter, robot->Y() );
	curPose[2] = CorrectTheta( robot->Theta() );
	
	x = WheelAverageX( robot->getWheelEncoderTotals( RI_WHEEL_RIGHT), robot->getWheelEncoderTotals( RI_WHEEL_LEFT ) ) / TICKS_PER_INCH;
	y =  WheelAverageY( robot->getWheelEncoderTotals( RI_WHEEL_RIGHT), robot->getWheelEncoderTotals( RI_WHEEL_LEFT ) ) / TICKS_PER_INCH;
	theta = robot->getWheelEncoderTotals( RI_WHEEL_REAR ) / ( PI * 11 ) / TICKS_PER_INCH;
	
	curWheelEnc[0] = x; // X
	curWheelEnc[1] = y; // Y
	curWheelEnc[2] = theta; // Theta
	
	rovioKalmanFilter( kf, curPose, curWheelEnc, predicted );
	
	return predicted;
}

// determines approx origin by averaging 10 readings
int SetOrigin()
{
	float xSum = 0;
	float ySum = 0;
	float thetaSum = 0;

	for ( int i=0; i<10; i++)
	{
		robot->update();
		xSum += FirFilter( xFilter, robot->X() );
		ySum += FirFilter( yFilter, robot->Y() );
		//thetaSum += robot->Theta();
	}
	
	xNSOrigin = xSum /10;
	yNSOrigin = ySum /10;
	//thetaCorrection = thetaSum /10;
	
	/*
	originDegree = 0;
	for(int i=0;i<10;i++){
		updateNorthStar(ACTION_TURN,1);
		originDegree += aveWeightedDegree;
	}
	originDegree = originDegree/10;//setting origin degree
	*/

	xEncCoord = 0;
	yEncCoord = 0;

	return OK;
}

int TurnTo( float target )
{
	float thetaCurrent, thetaSum;
	float offset = 3;
	float diff;//diff between current and target
	//PID *pid = new PID();
	bool multiple_sample = true;//indicate if we need to take multiple sample
	int turn_speed = 8;
	int turn_flag = RI_TURN_LEFT;//set initial turn flag to turn left


	
	printf("INSIDE TURNTO theta: %f\n", target);
	
	do {

		//thetaCurrent =  aveWeightedDegree;
		if(multiple_sample){//taking samples of current theta
			updateNorthStar(ACTION_TURN,1);
			thetaCurrent =  aveWeightedDegree;
			diff = MINABS(target-thetaCurrent, 360-thetaCurrent+target);//get diff between target and current

		}
		

		if(ABS(diff)<=offset)//break loop condition if diff is within a preset acceptable offset
			break;

		//if absolute value of diff >= 20
		if(ABS(diff) >= 20) {
			turn_flag = diff>0? RI_TURN_RIGHT_20DEG: RI_TURN_LEFT_20DEG;//set turn flag
			turn_speed = 4;

			if(ABS(diff)>=40)
				multiple_sample = false;//diff is greater than 40, then we need more 20degree turn
			else
				multiple_sample = true;//diff between 20 and 40, we need one more 20degree turn

			robot -> Move(turn_flag, turn_speed);
			diff += (diff>0?-1:1)*20;//update diff
		}
		//absolute value of diff < 20
		//doing adjustment
		else{
			multiple_sample = true;
			turn_flag = diff>0? RI_TURN_RIGHT: RI_TURN_LEFT;
			turn_speed = 6;

			//update sensor
			while(robot->update() != RI_RESP_SUCCESS) {
				printf("Failed to update sensor information!\n");
			}

			robot -> Move(turn_flag, turn_speed);

			while(robot->update() != RI_RESP_SUCCESS) {
				printf("Failed to update sensor information!\n");
			}
			
			if(!robot->IR_Detected()) {
				robot -> Move(RI_STOP, turn_speed);
			}

		}
		printf("target:%4.2f curr:%4.2f diff:%4.2F\n",target,thetaCurrent,diff);
		

	}
	while(1);


	/* old code
	
	robot->update();
	
	// get avg theta
	for (int i=0; i<10; i++)
	{
		thetaSum += CorrectTheta( robot->Theta() );
	}
	
	thetaCurrent = thetaSum / 10;

	// TODO: Handle case when sign changes
	
	printf("thetaCurrent: %.3f, theta: %.3f\n", thetaCurrent, theta );
	
	while ( thetaCurrent <= theta-0.05 && thetaCurrent > theta+0.05 )
	{
		printf("thetaCurrent: %.3f, theta: %.3f\n", thetaCurrent, theta );
		
		if ( thetaCurrent < theta )
		{
			robot->Move( RI_TURN_LEFT, RI_SLOWEST );
			
			robot->update();
			thetaCurrent = CorrectTheta( robot->Theta() );
		}
		
		else
		{
			robot->Move( RI_TURN_RIGHT, RI_SLOWEST );
			
			robot->update();
			thetaCurrent = CorrectTheta( robot->Theta() );
		}
	}
	*/
	return OK;
}

int MoveTo( int x, int y )
{
	yTotal = 0.0;
	int speed = RI_FASTEST;
	float rightEnc, leftEnc, yTicks, xTicks, ticks, newTheta;
	bool speedCheck = true;
	
	y = y - yEncCoord;
	x = x - xEncCoord;
	
	// convert x&y to ticks
	yTicks = y * TICKS_PER_INCH;
	xTicks = x * TICKS_PER_INCH;
	
	ticks = sqrt( pow(xTicks, 2) + pow(yTicks, 2) );
	
	robot->update();
	
	// move at top speed until w/in 20% of goal
	while ( 1 && !robot->IR_Detected() )
	{
		robot->Move( RI_MOVE_FORWARD, speed );
		robot->update();
		
		rightEnc = FirFilter( rightWheelFilter, robot->getWheelEncoder( RI_WHEEL_RIGHT ) );
		leftEnc = FirFilter( leftWheelFilter, robot->getWheelEncoder( RI_WHEEL_LEFT ) );
		yTotal +=  WheelAverageY( rightEnc, leftEnc );
		
		if ( yTotal >= (.8 * ticks ) && speedCheck )
		{
			speed = RI_SLOWEST;
			speedCheck = false;
		}
		
		if ( yTotal >= ticks )
			break;	

		//printf("yTotal: %.3f\n", yTotal);
	}

	return OK;
}

// handle ctrl + c
void QuitHandler( int s )
{
	printf( "Caught Ctrl + C. Dying a (more) graceful death\n" );
	
	delete(robot);
	
	#ifdef FILEDUMP
	rawDataFile.close();
	filterDataFile.close();
	#endif
	
	exit(-1);
}

int main(int argv, char **argc)
{
	// set up ctrl+c handler struct
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = QuitHandler;
	sigemptyset( &sigIntHandler.sa_mask );
	sigIntHandler.sa_flags = 0;
	sigaction( SIGINT, &sigIntHandler, NULL );

	// Make sure we have a valid command line argument
	if(argv <= 1) {
		std::cout << "Usage: robot_test <address of robot>" << std::endl;
		exit(-1);
	}

	// Setup the robot interface
	robot = new RobotInterface(argc[1],0);
	
	// initialize fir filters for each sensor
	if ( FAILED( InitializeFirFilters( robot ) ) )
	{
		std::cout << "Failed to initialize fir filters. Exiting" << std::endl;
		exit(-1);
	}
	
	
	// initialize kalman filter
	if ( FAILED( InitializeKalmanFilter( robot ) ) )
	{
		std::cout << "Failed to initialize kalman filter. Exiting" << std::endl;
		exit(-1);
	}
	
	
	#ifdef FILEDUMP
	rawDataFile.open("data/rawData.txt");
	filterDataFile.open("data/filterData.txt");
	#endif
	
	// set origin
	printf( "Gathering origin data...\n");
	SetOrigin();
	printf( "Origin set to (%.3f, %.3f) with heading %.3f\n", xNSOrigin, yNSOrigin, thetaCorrection );	
	
	printf("Navigating to 1st waypoint...\n");
	
	MoveTo( 0, 135 );
	xEncCoord = 0;
	yEncCoord = 135;
	
	TurnTo((float)225);
	
	MoveTo( 70, 95 );
	xEncCoord = 75;
	yEncCoord = 90;
	
	TurnTo((float)320);
	
	MoveTo( 123, 154 );
	xEncCoord = 123;
	yEncCoord = 154;
	
	printf( "Stopped at %.3f ticks (%.3f inches)\n", yTotal, (yTotal/TICKS_PER_INCH) );
	
	// Clean up
	delete(robot);
	
	#ifdef FILEDUMP
	rawDataFile.close();
	filterDataFile.close();
	#endif
	
	return 0;
}