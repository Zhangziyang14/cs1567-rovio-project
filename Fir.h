#ifndef _FIR_H_
#define _FIR_H_

/** * * * * * * * * * * * * * * * * * * * * * * *
* Fir.h											*
* simple fir filter code						*
* dmc - 03-04-08								*
* for cs 1567									*
* 												*
* Edited by: CJ McAllister, Yuxin Liu			* 
*												*
* Declares functions for use in FIR filtering	*
* * * * * * * * * * * * * * * * * * * * * * * * */

/*
Filter.h
simple fir filter code
Team: 
Date: 01022011
*/


class Fir {
public:
	Fir(float *, int ); //constructor
	Fir(const Fir * );	//copy constructor
	~Fir();	//destructor
	float getValue(float );	//return the next filtered value
	void showfilter();	//display filter info
	void reset(void);


private:	//private data
	float *coefficients;	//array of float coefficients
	unsigned next_sample;	//next sample position
	float *samples;	//sample array
	int size;	//size of the filter array
	bool initialized;
    void initialize(float first);

};

#endif