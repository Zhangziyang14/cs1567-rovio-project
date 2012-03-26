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

#include <vector>
using namespace std;

class Fir
{
public:
	Fir();
	~Fir();
	float getValue(float);
    void reset(void);

private:
    vector<float> coefficients; 	//coefficients[TAPS]
    vector<float>	samples;		//samples[TAPS]
    unsigned  next_sample;
    bool initialized;
    void initialize(float first);
};

#endif