#include <robot_if++.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <fstream>
#include <math.h>

#include "fir.h"
#include "function.h"
#include "robot.h"

#define FILEDUMP
//#define DUMP_NS

float xFinal, yFinal, thetaFinal, rightWheelFinal, leftWheelFinal, rearWheelFinal;
int currentRoom;

#ifdef FILEDUMP
std::ofstream rawDataFile;
std::ofstream filterDataFile;
#endif

// handle ctrl + c
void QuitHandler( int s )
{
	printf( "\nCaught Ctrl + C. Dying a (more) graceful death\n" );
	
	delete(robot);
	
	#ifdef FILEDUMP
	rawDataFile.close();
	filterDataFile.close();
	#endif
	
	exit(-1);
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
	rawDataFile.open("data/rawData.txt");
	filterDataFile.open("data/filterData.txt");
	#endif
	
	// set origin
	printf( "Gathering origin data...\n");
	SetOrigin();
	printf( "Origin set to (%.3f, %.3f) with heading %.3f\n", xOrigin, yOrigin, thetaCorrection );
	
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
		if ( yTotal >= 650 )
		{
			printf( "\n100 ticks reached! Final pose:\nxTotal: %f yTotal: %f\nX: %f Y: %f Theta: %f\n", xTotal, yTotal, xPos, yPos, thetaPos );
			break;
		}
		
		// check for room change
		if ( robot->RoomID() != currentRoom )
		{
			//printf("Pre-switch:\nRoom %d oldTheta: %.3f rawTheta: %.3f thetaCorrection: %.3f\n", currentRoom, thetaPos, robot->Theta(), thetaCorrection);
			RoomSwitch( thetaPos, robot->Theta() );
			currentRoom = robot->RoomID();
			
			//robot->update();
			//printf("Post-switch:\nRoom %d oldTheta: %.3f rawTheta: %.3f thetaCorrection: %.3f\n", currentRoom, thetaPos, robot->Theta(), thetaCorrection);
			
			
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
			thetaPos = CorrectTheta( robot->Theta() );
			
			xTotal += WheelAverageX( rightDist, leftDist ); 
			yTotal += WheelAverageY( rightDist, leftDist );
			
			printf( "Room %d signal: %d Theta: %.3f\n", currentRoom, robot->NavStrengthRaw(), thetaPos );
			
			#ifdef FILEDUMP
				#ifdef DUMP_NS
				rawDataFile << robot->X() << " " << robot->Y() << " " << robot->Theta() << "\n";
				filterDataFile << xPos << " " << yPos << " " << thetaPos << "\n";
				#else
				rawDataFile << robot->getWheelEncoder( RI_WHEEL_RIGHT ) << " " << robot->getWheelEncoder( RI_WHEEL_LEFT ) << "\n";
				filterDataFile << rightDist << " " << leftDist << " " << xTotal << " " << yTotal << "\n";
				#endif
			#endif
		}
	} while(1);

	// Clean up
	delete(robot);
	
	#ifdef FILEDUMP
	rawDataFile.close();
	filterDataFile.close();
	#endif
	
	return 0;
}