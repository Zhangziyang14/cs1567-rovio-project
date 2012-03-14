#ifndef _PID_H_
#define _PID_H_

/** * * * * * * * * * * * * * * * * * * * * *
* PID.h										*
* Created: 3/12/2012						*
* Authors: CJ McAllister, Brendan Liu		*
*											*
* Declares functions for use in PID control	*
* * * * * * * * * * * * * * * * * * * * * * */

#include <math.h>
#include <vector>
using namespace std;

class PID
{
public:
	PID();
	~PID();
	float control(float);
	
private:
	bool bInit;
    float err_PID[5]; // err_PID[5]
    float vel_PID;
    
    int currIndex;
    int prevIndex;
    
    float KP;
    float KI;
    float KD;
};

#endif