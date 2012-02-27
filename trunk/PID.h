#ifndef _PID_H_
#define _PID_H_

/**
 * PID.h
 * utility function
 */

#include <math.h> 

typedef struct 
{
    bool bInit;
    float *err_PID; // err_PID[5]
    float vel_PID;
    
    int currIndex;
    int prevIndex;
    
    float KP;
    float KI;
    float KD;
} PID;

PID *PID_Create(){
    int i;
    
    PID *p = (PID *) malloc(sizeof(PID));
	p->err_PID = (float *)calloc(5, sizeof(float));
    p->KP = 0.5;
    p->KI = 0.05;
    p->KD = 1.0;
    
    p->currIndex = 0;
    p->bInit = false;
    for (i=0; i<5; i++) {
        p->err_PID[i] = 0.0f;
    }
	
	return p;
    
}


// This funcition calculates the speed that the robot should move. 
// @param error -- How far the robot needs to travel.
// @return The full PID value.

float PID_control(PID *p, float error){
    float sum_error = 0;
	int prevIndex = p->currIndex - 1;
    
    if(p->bInit == false) {//if this is the first value pass in
		p->err_PID[p->currIndex] = error;
		p->currIndex = 1;//set current index to next pos
		p->bInit = true;//set flag to true
		return error;
    }
    

    
	// Wrap prevIndex if needed.
	if (prevIndex < 0) { 
        prevIndex = 4; 
    }
    
	// The current error.
	
	p->err_PID[p->currIndex] = error;
    
	// Calculate  the sum of the last five errors.
	for (int i = 0, j = p->currIndex; i < 5; i++) {
		sum_error += p->err_PID[j++];
		
		// Wrap the pointer to the start of the array.
		if (j == 5) { 
            j = 0; 
        }
	}
    
	//calculate return value
	float rt = (p->KP * p->err_PID[prevIndex]) + (p->KI * sum_error) + (p->KD * (fabs(p->err_PID[prevIndex]) - fabs(p->err_PID[p->currIndex])));
    
	// Set the currIndex to the nextImdex.
	p->currIndex++;
    
	// Wrap currIndex if needed.
	if (p->currIndex == 5) { 
        p->currIndex = 0; 
    }
    
	return rt;
    
}


#endif