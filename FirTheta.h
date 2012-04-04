/*
Filter.h
simple fir filter code
Team: 
Date: 01022011
*/

#ifndef _FIRTHETA_H_
#define _FIRTHETA_H_

class FirTheta {
public:
	FirTheta(float *, int ); //constructor
	FirTheta(const FirTheta * );	//copy constructor
	~FirTheta();	//destructor
	int getValue(int );	//return the next filtered value
	void showfilter();	//display filter info
	void reset(void);


private:	//private data
	float *coefficients;	//array of float coefficients
	int next_sample;	//next sample position
	int *samples;	//sample array
	int size;	//size of the filter array
	bool initialized;
    void initialize(float first);
	int prev;

};

#endif