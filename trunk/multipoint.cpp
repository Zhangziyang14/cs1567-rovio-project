#include <robot_if++.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>

#include "fir.h"
#include <stdlib.h>
#include <signal.h>
#include <fstream>

//#define FILEDUMP

#define PI 3.14159265

#define ANGLE_WHEEL_RIGHT 30
#define ANGLE_WHEEL_LEFT 150
#define ANGLE_WHEEL_REAR 90

#define OK 0
#define FAIL -1

#define FAILED(x) ((x) == OK) ? false : true

float xOrigin, yOrigin, thetaOrigin, rightWheelOrigin, leftWheelOrigin, rearWheelOrigin;
float xFinal, yFinal, thetaFinal, rightWheelFinal, leftWheelFinal, rearWheelFinal;

filter *xFilter = NULL;
filter *yFilter = NULL;
filter *thetaFilter = NULL;
filter *rightWheelFilter = NULL;
filter *leftWheelFilter = NULL;
filter *rearWheelFilter = NULL;

RobotInterface *robot;

#ifdef FILEDUMP
std::ofstream rawDataFile;
std::ofstream filterDataFile;
std::ofstream correctedFilterDataFile;
#endif
	
// initialize filters and prime coord filters w/ 3 readings
int InitializeFirFilters( RobotInterface *robot )
{
	if ( robot->update() == RI_RESP_SUCCESS ) {		
		// initialize filters
		xFilter = FirFilterCreate();
		yFilter = FirFilterCreate();
		thetaFilter = FirFilterCreate();
		rightWheelFilter = FirFilterCreate();
		leftWheelFilter = FirFilterCreate();
		rearWheelFilter = FirFilterCreate();
		
		// prime filters w/ 3 readings
		for ( int i=0; i<3; i++ )
		{
			robot->update();
			FirFilter( xFilter, robot->X() );
			FirFilter( yFilter, robot->Y() );
			FirFilter( thetaFilter, robot->Theta() );
		}
		
	}
	
	return OK;
}

// returns average of computed left and right encoder x-axis motion
float WheelAverageX( float rightEncoder, float leftEncoder )
{
	float rightFinal, leftFinal;
	rightFinal = rightEncoder * cos( ANGLE_WHEEL_RIGHT );
	leftFinal = leftEncoder * cos( ANGLE_WHEEL_LEFT );
	return (rightFinal + leftFinal) / 2;
}

// returns average of computed left and right encoder x-axis motion
float WheelAverageY( float rightEncoder, float leftEncoder )
{
	float rightFinal, leftFinal;
	rightFinal = rightEncoder * sin( ANGLE_WHEEL_RIGHT );
	leftFinal = leftEncoder * sin( ANGLE_WHEEL_LEFT );
	return (rightFinal + leftFinal) / 2;
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
		thetaSum += FirFilter( thetaFilter, robot->Theta() );
	}
	
	xOrigin = xSum /10;
	yOrigin = ySum /10;
	thetaOrigin = thetaSum /10;

	return OK;
}

int TurnTo( int theta )
{
	float thetaCurrent;

	robot->update();
	thetaCurrent = FirFilter( thetaFilter, robot->Theta() );

	// TODO: Handle case when sign changes
	
	if ( theta > thetaCurrent )
	{
		while ( theta > thetaCurrent )
		{
			robot->Move( RI_TURN_LEFT, RI_SLOWEST );
			
			robot->update();
			thetaCurrent = FirFilter( thetaFilter, robot->Theta() );
			printf( "thetaCurrent: %.3f\n", thetaCurrent );
		}
	}
	else
	{
		while ( theta < thetaCurrent )
		{
			robot->Move( RI_TURN_RIGHT, RI_SLOWEST );
			
			robot->update();
			thetaCurrent = FirFilter( thetaFilter, robot->Theta() );
			printf( "thetaCurrent: %.3f\n", thetaCurrent );
		}
	}
	
	return OK;
}

int MoveTo( int x, int y )
{
	float xPosCurrent, yPosCurrent, thetaCurrent, xDiff, yDiff, m, newTheta;
	
	robot->update();
	xPosCurrent = FirFilter( xFilter, robot->X() );
	yPosCurrent = FirFilter( yFilter, robot->Y() );
	thetaCurrent = FirFilter( thetaFilter, robot->Theta() );
	
	//determine theta for direct path
	xDiff = xPosCurrent - x;
	yDiff = yPosCurrent - y;
	
	// handle undefined slope case
	if ( xDiff == 0 )
	{
		printf("Undefined slope. Exiting.\n");
		exit(-1);
	}
	else
	{
		m = yDiff / xDiff;
		newTheta = atan( m );
	}
	
	printf("Slope: %.3f New Theta: %.3f\n", m, newTheta );
	
	//TurnTo( newTheta );
	
	// go to x coord first
	while( 1 )
	{
		robot->update();
		xPosCurrent = FirFilter( xFilter, robot->X() );
		yPosCurrent = FirFilter( yFilter, robot->Y() );
		
		printf( "xCur: %.3f, xFin: %d\n", xPosCurrent, x );
		
		// robot behind desired x
		if ( xPosCurrent < x - 100 )
		{
			robot->Move( RI_MOVE_FORWARD, RI_SLOWEST );
		}
		
		// robot in front of desired x
		else if ( xPosCurrent >= x + 100 )
		{
			robot->Move( RI_MOVE_BACKWARD, RI_SLOWEST );
		}
		
		// robot within +-100 of destination
		else
			break;
	}
	
	printf("Y TIME! :D\n");
	
	//robot is below desired y
	if ( yPosCurrent < y - 10 )
	{
		// TURN AND MOVE
	}
	
	// robot is above desired y
	else if ( yPosCurrent >= y + 10 )
	{
		// TURN AND MOVE
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
	correctedFilterDataFile.close();
	#endif
	
	exit(-1);
}

// corrects the x coordinate based on theta
float ThetaCorrectionX( float x, float y, float theta )
{
	return ( x * cos(theta) ) - ( y * sin(theta) );
}

// corrects the y coordinate based on theta
float ThetaCorrectionY( float x, float y, float theta )
{
	return ( x * sin(theta) ) + ( y * cos(theta) );
}

int main(int argv, char **argc)
{
	float xTotal = 0.0, yTotal = 0.0;
	float rightDist, leftDist, rearDist;
	float xPos, yPos, thetaPos;
	float xPosCorrected, yPosCorrected;
	
	int w1x, w1y;
	
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
	
	// initialize origins and fir filters for each sensor
	if ( FAILED( InitializeFirFilters( robot ) ) )
	{
		std::cout << "Failed to initialize filters. Exiting" << std::endl;
		exit(-1);
	}
	
	#ifdef FILEDUMP
	rawDataFile.open("rawData.txt");
	filterDataFile.open("filterData.txt");
	correctedFilterDataFile.open("correctedFilterData.txt");
	#endif
	
	// set origin
	printf( "Gathering origin data...\n");
	SetOrigin();
	printf( "Origin set to (%.3f, %.3f) with heading %.3f\n", xOrigin, yOrigin, thetaOrigin );
	
	/*
	// get waypoint coords
	printf( "Enter x, y of 1st waypoint (space delimeted): " );
	scanf( "%d %d", &w1x, &w1y );
	printf( "Moving to position (%d, %d)\n", w1x, w1y );	

	MoveTo( w1x, w1y );
	*/
	
	// Action loop
	do {
		// Update the robot's sensor information
		if(robot->update() != RI_RESP_SUCCESS) {
			std::cout << "Failed to update sensor information!" << std::endl;
			break;
		}
		
		//break at 3 meters
		if ( xTotal >= 500 )
		{
			printf( "\n3m reached! Final pose:\nxTotal: %f yTotal: %f\nX: %f Y: %f Theta: %f\n", xTotal, yTotal, xPos, yPos, thetaPos );
			break;
		}
		
		// Move unless there's something in front of the robot
		if(!robot->IR_Detected())
		{
			robot->Move( RI_MOVE_FORWARD, RI_SLOWEST );
			
			rightDist = FirFilter( rightWheelFilter, robot->getWheelEncoder( RI_WHEEL_RIGHT ) );
			leftDist = FirFilter( leftWheelFilter, robot->getWheelEncoder( RI_WHEEL_LEFT ) );
			rearDist = FirFilter( rearWheelFilter, robot->getWheelEncoder( RI_WHEEL_REAR ) );
			
			xPos = FirFilter( xFilter, robot->X() );
			yPos = FirFilter( yFilter, robot->Y() );
			thetaPos = FirFilter( thetaFilter, robot->Theta() );
			
			xTotal += WheelAverageX( rightDist, leftDist ); 
			yTotal += WheelAverageY( rightDist, leftDist );
			
			//printf( "X: %f, Y: %f, THETA: %f, SIG: %d, ROOM: %d\n", xPos, yPos, thetaPos, robot->NavStrengthRaw(), robot->RoomID() );
			//printf( "RAW X: %d FIR X: %f\n", robot->X(), xPos );
			
			// theta correction
			xPosCorrected = ThetaCorrectionX( xPos, yPos, thetaPos );
			yPosCorrected = ThetaCorrectionY( xPos, yPos, thetaPos );
			
			//printf("theta: %10.3f | x: %10.3f y: %10.3f | x: %10.3f y: %10.3f\n", thetaPos, xPos, yPos, xPosCorrected, yPosCorrected );
			
			#ifdef FILEDUMP
			rawDataFile << robot->X() << " " << robot->Y() << "\n";
			filterDataFile << xPos << " " << yPos << "\n";
			correctedFilterDataFile << xPosCorrected << " " << yPosCorrected << "\n";
			#endif
		}
	} while(1);

	// Clean up
	delete(robot);
	
	#ifdef FILEDUMP
	rawDataFile.close();
	filterDataFile.close();
	correctedFilterDataFile.close();
	#endif
	
	return 0;
}