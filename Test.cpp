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
 

int main(int argv, char **argc) {
 
	// Make sure we have a valid command line argument.
 
	RobotInterface *robot = new RobotInterface(argc[1], 1);
	Camera *cam = new Camera();
	cam->InitCamera(robot);

	/*
	Robot *robot = new Robot(argc[1]);
	robot->init();
	printf("done.\n");
	robot->test();
	*/

	cam->CamCenter();
 
	//clean up
	//delete(robot);
	return 0;
}
