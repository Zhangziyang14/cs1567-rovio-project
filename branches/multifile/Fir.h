/** * * * * * * * * * * * * * * * * * * * * * * *
* Fir.h											*
* simple fir filter code						*
* dmc - 03-04-08								*
* for cs 1567									*
* 												*
* Edited by: CJ McAllister, Brendan Liu			* 
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
	float filter( float );

private:
  vector<float> coefficients; 	//coefficients[TAPS]
  vector<float>	samples;		//samples[TAPS]
  unsigned  next_sample;
};