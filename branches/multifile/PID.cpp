/** * * * * * * * * * * * * * * * * * * * * *
* PID.cpp									*
* Created: 3/12/2012						*
* Authors: CJ McAllister, Brendan Liu		*
*											*
* Provides implementation of PID control	*
* * * * * * * * * * * * * * * * * * * * * * */

#include "PID.h"

// Constructor
// Initializes values for PID control
PID::PID()
{
    KP = 0.5;
    KI = 0.05;
    KD = 1.0;
    
    currIndex = 0;
    bInit = false;
	
    for (int i=0; i<5; i++) {
        err_PID.push_back(0.0f);
    }
}

// Destructor
// Frees memory of PID object
PID::~PID()
{
	delete &err_PID;
}


// This funcition calculates the speed that the robot should move. 
// @param error -- How far the robot needs to travel.
// @return The full PID value.

float PID::control(float error)
{
    float sum_error = 0;
	int prevIndex = currIndex - 1;
    
    if(bInit == false) {//if this is the first value pass in
		err_PID[currIndex] = error;
		currIndex = 1;//set current index to next pos
		bInit = true;//set flag to true
		return error;
    }
    

    
	// Wrap prevIndex if needed.
	if (prevIndex < 0) { 
        prevIndex = 4; 
    }
    
	// The current error.
	
	err_PID[currIndex] = error;
    
	// Calculate  the sum of the last five errors.
	for (int i = 0, j = currIndex; i < 5; i++) {
		sum_error += err_PID[j++];
		
		// Wrap the pointer to the start of the array.
		if (j == 5) { 
            j = 0; 
        }
	}
    
	//calculate return value
	float rt = (KP * err_PID[prevIndex]) + (KI * sum_error) + (KD * (fabs(err_PID[prevIndex]) - fabs(err_PID[currIndex])));
    
	// Set the currIndex to the nextImdex.
	currIndex++;
    
	// Wrap currIndex if needed.
	if (currIndex == 5) { 
        currIndex = 0; 
    }
    
	return rt;
    
}
