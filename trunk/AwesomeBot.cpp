#include "robot.h"
#include "function.h"

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
		
	}
	
	return OK;
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
		thetaSum += robot->Theta();
	}
	
	xOrigin = xSum /10;
	yOrigin = ySum /10;
	thetaCorrection = thetaSum /10;

	printf( "thetaCorrection:	%.3f\n", thetaCorrection );
	printf( "0'd theta:	%.3f\n", CorrectTheta( thetaCorrection ) );
	
	currentRoom = robot->RoomID();

	return OK;
}

int TurnTo( int theta )
{
	float thetaCurrent, thetaSum;
	
	robot->update();
	
	// get avg theta
	for (int i=0; i<10; i++)
	{
		thetaSum += CorrectTheta( robot->Theta() );
	}
	
	thetaCurrent = thetaSum / 10;

	// TODO: Handle case when sign changes
	
	if ( theta > thetaCurrent )
	{
		while ( theta > thetaCurrent )
		{
			robot->Move( RI_TURN_LEFT, RI_SLOWEST );
			
			robot->update();
			thetaCurrent = CorrectTheta( robot->Theta() );
			printf( "thetaCurrent: %.3f\n", thetaCurrent );
		}
	}
	else
	{
		while ( theta < thetaCurrent )
		{
			robot->Move( RI_TURN_RIGHT, RI_SLOWEST );
			
			robot->update();
			thetaCurrent = CorrectTheta( robot->Theta() );
			printf( "thetaCurrent: %.3f\n", thetaCurrent );
		}
	}
	
	return OK;
}

int MoveTo( int x, int y )
{
	float xPosCurrent, yPosCurrent, thetaCurrent, xDiff, yDiff, m, newTheta;
	
	// go to x coord first
	printf("Moving to x coord...\n");
	while( 1 )
	{
		robot->update();
		xPosCurrent = FirFilter( xFilter, robot->X() );
		yPosCurrent = FirFilter( yFilter, robot->Y() );
		thetaCurrent = CorrectTheta( robot->Theta() );
		
		printf( "xCur: %.3f, xFin: %d\n", xPosCurrent, x );
		
		// robot behind desired x
		if ( xPosCurrent < x - 50 )
		{
			robot->Move( RI_MOVE_FORWARD, RI_SLOWEST );
		}
		
		// robot in front of desired x
		else if ( xPosCurrent >= x + 50 )
		{
			robot->Move( RI_MOVE_BACKWARD, RI_SLOWEST );
		}
		
		// robot within +-100 of destination
		else
			break;
	}
	return OK;
}