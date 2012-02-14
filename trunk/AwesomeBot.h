/**
* AwesomeBot.h
* 
* 
*/

#ifndef _AWESOMEBOT_H_
#define _AWESOMEBOT_H_
#include "robot_if++.h"
#include "stdio.h"
#include "string"
#include "iostream"

extern "C" {
	#include "Filter.h"
}

using namespace std;

class AwesomeBot {
/*private variables*/
private:
	float xOrigin, yOrigin, rightWheelOrigin, leftWheelOrigin, rearWheelOrigin;

	filter *xFilter = NULL;
	filter *yFilter = NULL;
	filter *rFilter = NULL;
	filter *lFilter = NULL;
	filter *bFilter = NULL;

	RobotInterface *robot;

public: /*public member functions*/
	AwseomeBot(string);
	~AwseomeBot();
	int InitializeFirFilters( RobotInterface *robot );
	int SetOrigin();
	int TurnTo( int theta );
	int MoveTo( int x, int y );

}

#endif
