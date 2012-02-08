/* 
   fir.h
   simple fir filter code
 * dmc - 03-04-08
 * for cs 1567
 
*/

#define TAPS  4 // how many filter taps

typedef struct 
{
  float coefficients[TAPS];
  unsigned  next_sample;
  float samples[TAPS];
} filter;

/* firFilterCreate()
 * creates, allocates,  and iniitializes a new firFilter
 */
filter *FirFilterCreate()
{
  int i;
  filter *f = (filter *) malloc(sizeof(filter));
  for (i=0; i<TAPS; i++) {
    f->samples[i] = 0;
    f->coefficients[i] = 1. /(float) TAPS; // user must set coef's
  }
  f->next_sample = 0;
}

/* firFilter 
 * inputs take a filter (f) and the next sample (val)
 * returns the next filtered sample
 * incorporates new sample into filter data array
 */

float FirFilter(filter *f, float val)
{
  float sum =0;
  int i,j;

  /* assign  new value to "next" slot */
  f->samples[f->next_sample] = val;

  /* calculate a  weighted sum
     i tracks the next coeficeint
     j tracks the samples w/wrap-around */
  for( i=0,j=f->next_sample; i<TAPS; i++) {
    sum += f->coefficients[i]*f->samples[j++];
    if(j==TAPS)  j=0;
  }
  if(++(f->next_sample) == TAPS) f->next_sample = 0;
  return(sum);
}