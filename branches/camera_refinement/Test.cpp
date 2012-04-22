/** * * * * * * * * * * * * * * * * * * * * *
 * test.cpp									*
 * Created: 3/12/2012						*
 * Authors: CJ McAllister, Yuxin Liu         *
 *											*
 * * * * * * * * * * * * * * * * * * * * * * */
 
#include "robot_if++.h"
#include "Robot.h"
#include <iostream>
#include "string"
#include "ostream"

using namespace std;
 

int main(int argv, char **argc) {
 
	// Make sure we have a valid command line argument.
	string robotName(argc[1]);
	RobotInterface *ri = new RobotInterface(argc[1], 1);

	Robot *robot = new Robot(argc[1]);
	robot->Init();

	robot->test();

	cout << "Test Complete!" << endl;

	return 0;
}
