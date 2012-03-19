/** * * * * * * * * * * * * * * * * * * * * * * *
* Fir.cpp											*
* simple fir filter code						*
* dmc - 03-04-08								*
* for cs 1567									*
* 												*
* Edited by: CJ McAllister, Yuxin Liu			* 
*												*
* Declares functions for use in FIR filtering	*
* * * * * * * * * * * * * * * * * * * * * * * * */

#include "Fir.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;

/**
* public constructor
* @coe is the array of coefficients
* @size is the size of coefficient array or TAPS
*/
Fir::Fir(float *coe, int n): size(n), next_sample(0){	//size is n, next_sample is 0
	coefficients = (float*) malloc (sizeof(float)*n);	//allocate space
	samples = (float*) malloc (sizeof(float)*n);	//allocate space
	for(int i = 0;i<n;i++) {//copy each coe value
		coefficients[i] = coe[i];	
		samples[i]=0;	//all sample initiate to 0
	}

	initialized = false;
}

/*copy constructor, make deep copies*/
Fir::Fir(const Fir* filter): next_sample(0){
	this->size = filter->size;
	coefficients = (float*) malloc (sizeof(float)*size);
	for(int i = 0;i<size;i++) {
		coefficients[i] = filter->coefficients[i];
		samples[i]=0;
	}
	initialized = false;
}	

/**
* destructor
*/
Fir::~Fir() {
	delete(coefficients);	//free coefficients
	delete(samples);	//free sample array
}

/* firFilter
 * inputs take a filter (f) and the next sample (val)
 * returns the next filtered sample
 * incorporates new sample into filter data array
 */
float Fir::getValue( float val)
{

  float sum =0;
  int i,j;

  if(!initialized){
	  initialize(val);
  }

  /* assign new value to "next" slot */
  samples[next_sample] = val;
  /* calculate a weighted sum
     i tracks the next coefficeint
     j tracks the samples w/wrap-around */
  for( i=0,j=next_sample; i<size; i++) {
    sum += coefficients[i]*samples[j--];
    if(j==-1) j=size-1;
  }
  if(++(next_sample) == size) next_sample = 0;
  return(sum);
}

void Fir::initialize( float val){
		for(int i=0;i<size;i++) {
			samples[i]=val;
		}
		initialized = true;
}


void Fir::showfilter() {
	float sum=0;
	printf("\n-------------------------------Filter--------------------------\n");
	for(int i=0;i<size;i++) {
		printf("%f\n",coefficients[i]);
		sum+=coefficients[i];
	}
	printf("sum=%f\n",sum);
	printf("-------------------------------Filter--------------------------\n");
}

void Fir::reset(){
	initialized = false;
}