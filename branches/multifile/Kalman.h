#ifndef _KALMAN_H_
#define _KALMAN_H_

/** * * * * * * * * * * * * * * * * * * * * * * *
* Kalman.h										*
* Created: 3/12/2012							*
* Authors: CJ McAllister, Brendan Liu			*
*												*
* Declares functions for use in Kalman filter	*
* * * * * * * * * * * * * * * * * * * * * * * * */

#include <clapack.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
using namespace std;

class Kalman
{
public:
	Kalman( float *, float *,  int );
	~Kalman();
	void rovioKalmanFilter( float *, float *, float * );
	void rovioKalmanFilterSetVelocity( float * );
	void rovioKalmanFilterSetUncertainty(  float * );

private:
	float Q				[FILTER_SIZE * FILTER_SIZE];
	float R1			[FILTER_SIZE * FILTER_SIZE];
	float R2			[FILTER_SIZE * FILTER_SIZE];
	float Phi			[FILTER_SIZE * FILTER_SIZE]; 
	float residual_s1	[FILTER_SIZE];
	float residual_s2	[FILTER_SIZE];
	float W1			[FILTER_SIZE * FILTER_SIZE];
	float W2			[FILTER_SIZE * FILTER_SIZE];
	float current_state	[FILTER_SIZE];
	float P				[FILTER_SIZE * FILTER_SIZE]; 
};

#endif