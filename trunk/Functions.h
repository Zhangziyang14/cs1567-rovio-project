#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

/**
 * Function.h
 * utility function
 */

#include <math.h> 
 
#define COS_R 0.8660254            // cos(30).
#define COS_L -0.8660254		   // cos(150).
#define SIN 0.5                    // sin(30) and sin(150).
#define PI 3.1415926
#define DIAMETER 29
#define TODEGREE(x) ((float)((x+PI)*57.29578)) //turn theta into degree
#define ABS(x) ((x)>0?(x):-(x))
#define TORADIAN(x) ((float)(x*0.0174533) - PI) //turn degree to radian
#define MINABS(x,y) (ABS(x)<ABS(y)?(x):(y))//return the one with lower absolute value
#define SIGNAL_CHECK while(robot->update()!=RI_RESP_SUCCESS){printf("Failed to updated sensor information\n");}

#define NS_PER_CM 20
#define TICKS_PER_CM 1.8

#define ANGLE_WHEEL_RIGHT 0.523598776
#define ANGLE_WHEEL_LEFT 2.61799388
#define ANGLE_WHEEL_REAR 1.57079633

// the values in these arrays correspond to the x-y correction values for a given room
// e.g. xCorrections[2] will give the correction factor for the x coordinate in room 2
// NOTE: odd-numbered rooms will have their x and y coords reversed
float xCorrections[6] = {0, 0, -0.0876, -0.2035, 0, 0};
float yCorrections[6] = {0, 0, -0.3536, -0.1758, 0, 0};
float xOrigins[6] = {0, 0, -11000, 0, 0, 0};
float yOrigins[6] = {0, 0, -2300, 0, 0, 0};

float currDist = 0;


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

float CorrectTheta(float oldTheta, int roomID){
	float newTheta;
	
	switch(roomID) {
		case 2: //room 2
			newTheta = 360+255 - TODEGREE(oldTheta);
			break;
		case 3: //room3
			newTheta = 360+180 - TODEGREE(oldTheta);
			break;
		case 4: //room 4
			newTheta = 360+275 - TODEGREE(oldTheta);
			break;
		case 5: //room 5
			newTheta = 360+4 - TODEGREE(oldTheta);
			break;
		default:
			//printf("room %d, Man, are you sure you are not in outer space!\n",roomID);
			return 0;
	}
	
	return TORADIAN(fmod( newTheta, (float)360 ));
}

// corrects the skew in X NS coords and converts to centimeters
float CorrectXtoCM( float xCoord, int roomID )
{
	float correctedX, translatedX;
	correctedX = xCoord - ( currDist * xCorrections[roomID] );
	translatedX = correctedX - xOrigins[roomID];
	return translatedX / NS_PER_CM;
}

// corrects the skew in Y NS coords and converts to centimeters
float CorrectYtoCM( float yCoord, int roomID )
{
	float correctedY, translatedY;
	correctedY = yCoord - ( currDist * yCorrections[roomID] );
	translatedY = correctedY - yOrigins[roomID];
	return translatedY / NS_PER_CM;
}

// gets distance from current and previous x and y coordinates
float GetDistance( float prevX, float curX, float prevY, float curY )
{
	float deltaX, deltaY;
	
	deltaX = prevX - curX;
	deltaY = prevY - curY;
	
	return sqrt( pow((deltaX), 2) + pow((deltaY), 2) );
}

#endif