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
	if(argv <= 1) {
		cout << "Usage: robot_test <address of robot>" << std::endl;
        exit(-1);
	}
 

 
	Robot *robot = new Robot(argc[1]);

	
	robot->Init();
    robot->ReadData();
	
    //robot->MoveTo(0,336);//base 1
	//robot->TurnTo(145);
	
 
	//clean up
	delete(robot);
	return 0;
}
