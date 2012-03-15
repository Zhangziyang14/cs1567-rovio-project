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
	int major, minor;
	IplImage *image = NULL, *hsv = NULL, *threshold = NULL;
	squares_t *squares, *biggest, *sq_idx;
	CvPoint pt1, pt2;
	int sq_amt;
 
	// Make sure we have a valid command line argument.
	if( argv <= 1 ){
		exit(-1);
	}
 
	// create and initialize robot
	Robot *robot = new Robot(argc[1]);
	robot->Init();
	robot->InitCamera();

	robot->ShowImage();
 
}
