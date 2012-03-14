/** * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* Fir.cpp																*
* Created: 3/12/2012													*
* Authors: CJ McAllister, Yuxin Liu									*
*																		*
* Provides implementation of FIR filtering functions defined in Fir.h	*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Fir.h"

#define TAPS 4 // how many filter taps

// Constructor
// Iniitializes a new Fir filter
Fir::Fir()
{
	for (int i=0; i<TAPS; i++)
	{
		samples.push_back(0);
		coefficients.push_back(1. /(float) TAPS); // user must set coef's
	}
	
	next_sample = 0;
    initialized = false;
}

// Destructor
// Frees memory of Fir object
Fir::~Fir()
{
	delete &samples;
	delete &coefficients;
}

/* filter()
 * inputs take a filter (f) and the next sample (val)
 * returns the next filtered sample
 * incorporates new sample into filter data array
 */
float Fir::getValue(float val)
{
	float sum = 0;

	/* assign  new value to "next" slot */
    if (initialized == false) {
        initialize(val);
    }
    else{
        samples[next_sample] = val;
    }
	/* calculate a  weighted sum
	 i tracks the next coeficeint
	 j tracks the samples w/wrap-around */
	for( int i=0, j=next_sample; i<TAPS; i++)
	{
		sum += coefficients[i] * samples[j++];
		
		if(j==TAPS)
			j=0;
	}

	if( ++next_sample == TAPS)
		next_sample = 0;
	
	return sum;
}

void Filter::initialize(float first) {
    int i;
    
    initialized = true;
    
    for (i = 0; i < taps; i++) {
        samples[i] = first;
    }
}

void Filter::reset(void) {
    initialized = false;
}