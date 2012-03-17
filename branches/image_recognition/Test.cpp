/** * * * * * * * * * * * * * * * * * * * * *
 * test.cpp									*
 * Created: 3/12/2012						*
 * Authors: CJ McAllister, Yuxin Liu         *
 *											*
 * * * * * * * * * * * * * * * * * * * * * * */
 
#include "robot_if++.h"
#include "Robot.h"
#include "stdio.h"
#include "string"
#include "ostream"

using namespace std;
 

int main(int argv, char **argc)
{ 
	// Make sure we have a valid command line argument.
	if( argv <= 1 ){
		exit(-1);
	}
 
	// create and initialize robot
	Robot *robot = new Robot(argc[1]);
	robot->Init();
	robot->InitCamera();

	robot->CamNav();
 
}
