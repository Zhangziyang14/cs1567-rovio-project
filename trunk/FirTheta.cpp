#include "FirTheta.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;

#define TODEGREE(x) ((float)((x)*57.29578)) //turn radian into degree
#define TORADIAN(x) ((float)(x*0.0174533)) //turn degree to radian
#define PI 3.1415926

/**
* public constructor
* @coe is the array of coefficients
* @size is the size of coefficient array or TAPS
*/
FirTheta::FirTheta(float *coe, int n): size(n), next_sample(0){	//size is n, next_sample is 0
	coefficients = (float*) malloc (sizeof(float)*n);	//allocate space
	samples = (int*) malloc (sizeof(int)*n);	//allocate space
	for(int i = 0;i<n;i++) {//copy each coe value
		coefficients[i] = coe[i];	
		samples[i]=0;	//all sample initiate to 0
		
	}
	prev = 0;

	initialized = false;
}

/*copy constructor, make deep copies*/
FirTheta::FirTheta(const FirTheta* filter): next_sample(0){
	this->size = filter->size;
	coefficients = (float*) malloc (sizeof(float)*size);
	for(int i = 0;i<size;i++) {
		coefficients[i] = filter->coefficients[i];
		samples[i]=0;
		
	}
	prev = 0;
	initialized = false;
}	

/**
* destructor
*/
FirTheta::~FirTheta() {
	delete(coefficients);	//free coefficients
	delete(samples);	//free sample array
}

/* firFilter
 * inputs take a filter (f) and the next sample (val)
 * returns the next filtered sample
 * incorporates new sample into filter data array
 */
int FirTheta::getValue( int val)
{

  int sum =0;
  int i,j;
  int toSample;
  int result;


  if(!initialized){
	  initialize(val);
  }

  toSample = val;

  if(prev - val>200){
	  toSample += 360; 
  }
  if(prev - val<-200){
	  for(i=0;i<size;i++){
		  samples[i]+= 360;
	  }
  }

  /* assign new value to "next" slot */
  samples[next_sample] = toSample;
  prev = toSample;

  
  /* calculate a weighted sum
     i tracks the next coefficeint
     j tracks the samples w/wrap-around */
  for( i=0,j=next_sample; i<size; i++) {
    sum += coefficients[i]*samples[j--];
    if(j==-1) j=size-1;
  }
  if(++(next_sample) == size) next_sample = 0;
  
   
  result = sum % 360;
  //showfilter();
  //printf("cur:%3d prev:%3d fir:%3d result:%3d next:%d \n",val, prev, toSample,result,next_sample);
  return(result);
} 

void FirTheta::initialize( float val){
		for(int i=0;i<size;i++) {
			samples[i]=val;
			prev = val;
		}
		initialized = true;
}


void FirTheta::showfilter() {
	float sum=0;
	printf("\n-------------------------------Filter--------------------------\n");
	for(int i=0;i<size;i++) {
		printf("%d\n",samples[i]);
		sum+=coefficients[i];
	}
	printf("sum=%f\n",sum);
	printf("next=%d\n",next_sample);
	printf("-------------------------------Filter--------------------------\n");
}

void FirTheta::reset(){
	initialized = false;
}