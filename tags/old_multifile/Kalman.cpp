/** * * * * * * * * * * * * * * * * * * * * *
* Kalman.cpp								*
* Created: 3/12/2012						*
* Authors: CJ McAllister, Brendan Liu		*
*											*
* Provides implementation of Kalman filter	*
* * * * * * * * * * * * * * * * * * * * * * */


#define FILTER_SIZE 9

#define NORTHSTAR_UNCERTAINTY_X  .1 	// this is the uncertainty of the northstar readings, we are providing
#define NORTHSTAR_UNCERTAINTY_Y  .2 	// you with the capability to independently set the x,y,and theta
#define NORTHSTAR_UNCERTAINTY_TH .05

#define WHEELENC_UNCERTAINTY_X  .05 	// this is the uncertainty of the wheel encoder readings, 
#define WHEELENC_UNCERTAINTY_Y  .05 	// 
#define WHEELENC_UNCERTAINTY_TH .2

#define PROCESS_UNCERTAINTY_X  .05 		// this is the uncertainty about whether the robot will obey the model
#define PROCESS_UNCERTAINTY_Y  .05
#define PROCESS_UNCERTAINTY_TH .05

#define INIT_VAL	0.05

#define diag(x, val)	{\
	for(i=0; i<FILTER_SIZE; i++) {\
		x[i*FILTER_SIZE + i] = val;\
	}\
}

#define pmat(x) {\
	printf("===============================================================\n");\
	for(i=0; i<FILTER_SIZE; i++) {\
		for(j=0; j<FILTER_SIZE; j++) {\
			printf("%f ", x[i * FILTER_SIZE + j]);\
		}\
		printf("\n");\
	}\
}

#define ROWCOL(I,J) (I*FILTER_SIZE+J)

#include "Kalman.h"


// Contsrtuctor
Kalman::Kalman()
{    
  // zero the filter arrays
  memset(Q, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  memset(Phi, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  memset(R1, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  memset(R2, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  memset(W1, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  memset(W2, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  memset(P, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
}

// Destructor
// Frees memory of Kalman object
Kalman::~Kalman()
{
	delete Q;
	delete R1;
	delete R2;
	delete Phi;
	delete residual_s1;
	delete residual_s2;
	delete W1;
	delete W2;
	delete current_state;
	delete P;
}


/* Initialize the filter */
void Kalman::initialize( float *initPose, float *velocity, int deltat )
{  
    int i;

  /* Initialize the model array */
  diag(Phi,1);
  Phi[ROWCOL(0,3)]=deltat;
  Phi[ROWCOL(1,4)]=deltat;
  Phi[ROWCOL(2,5)]=deltat;
  Phi[ROWCOL(3,6)]=deltat;
  Phi[ROWCOL(4,7)]=deltat;
  Phi[ROWCOL(5,8)]=deltat;

  // initialize the uncertainty array
  Q[0] = PROCESS_UNCERTAINTY_X;
  Q[FILTER_SIZE+1]   = PROCESS_UNCERTAINTY_Y;
  Q[2*FILTER_SIZE+2] = PROCESS_UNCERTAINTY_TH;

  R1[0] = NORTHSTAR_UNCERTAINTY_X;
  R1[FILTER_SIZE+1]   = NORTHSTAR_UNCERTAINTY_Y;
  R1[2*FILTER_SIZE+2] = NORTHSTAR_UNCERTAINTY_TH;

  R2[0] = WHEELENC_UNCERTAINTY_X;
  R2[FILTER_SIZE+1]   = WHEELENC_UNCERTAINTY_Y;
  R2[2*FILTER_SIZE+2] = WHEELENC_UNCERTAINTY_TH;

  // initialize the state
  current_state[0] = initPose[0];
  current_state[1] = initPose[1];
  current_state[2] = initPose[2];
  current_state[3] = velocity[0];
  current_state[4] = velocity[1];
  current_state[5] = velocity[2];
  current_state[6] = 0;
  current_state[7] = 0;
  current_state[8] = 0;
}

void Kalman::rovioKalmanFilter( float *meas_S1, float *meas_S2, float *predicted ) {
	int i;

	/* Variables for matrix inversion */	
	int mtrx_sz = FILTER_SIZE;
	int info, lwork;
	int *ipiv = NULL;
	float *work = NULL;

	// some temp variables that we need
	float temp[FILTER_SIZE * FILTER_SIZE];
	float temp2[FILTER_SIZE * FILTER_SIZE];
	float temp3[FILTER_SIZE * FILTER_SIZE];
	float Ptmp[FILTER_SIZE * FILTER_SIZE];
	float eye[FILTER_SIZE * FILTER_SIZE];
	float new_state[FILTER_SIZE];
	
	/* Clear the temp matricies that aren't directly overwritten */
	memset(eye, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);

	/* Allocate ipiv and work */
	ipiv = (int *) malloc(mtrx_sz * sizeof(int));
	lwork = FILTER_SIZE * FILTER_SIZE;
	work = (float *) malloc(sizeof(float) * lwork);
	if(!ipiv || !work) {
		if(ipiv)
			free(ipiv);
		if(work)
			free(work);
		printf("Error allocating memory!\n");
		return;
	}

	/**** 2. Propagate the Covariance Matrix ****/
	/* temp2 = Phi * P */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f, 
		Phi, FILTER_SIZE, P, FILTER_SIZE, 0.0f, temp2, FILTER_SIZE); 

	/* temp = temp2 * Phi' */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp2, FILTER_SIZE, Phi, FILTER_SIZE, 0.0f, temp, FILTER_SIZE);

	/* P = temp + Q */
	for(i=0; i<FILTER_SIZE*FILTER_SIZE; i++) 
		P[i] = temp[i] + Q[i]; 

	/**** 3. Propagate the model track estimate ****/
	/* new_state = Phi * current_state */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, 1, FILTER_SIZE, 1.0f,
		Phi, FILTER_SIZE, current_state, 1, 0.0f, new_state, 1);

	for(i=0; i<3; i++) {
	  residual_s1[i] = meas_S1[i] - new_state[i];
	  residual_s2[i] = meas_S2[i] - new_state[i];
	}
	for(i=3; i<9; i++) {
	  residual_s1[i] = 0;
	  residual_s2[i] = 0;
	}

	/* temp = P + R1 */
	for(i=0; i<FILTER_SIZE * FILTER_SIZE; i++) 
	  temp[i] = P[i] + R1[i];

	/* Invert temp by first performing an LU Decomposition */
	info = clapack_sgetrf(CblasRowMajor, mtrx_sz, mtrx_sz, temp, mtrx_sz, ipiv);

	/* Now, invert given the LU decomposition */
	info = clapack_sgetri(CblasRowMajor, mtrx_sz, temp, mtrx_sz, ipiv);

	/* W1 = P * temp */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		P, FILTER_SIZE, temp, FILTER_SIZE, 0.0f, W1, FILTER_SIZE);
	
	/* temp = P + R2 */
	for(i=0; i<FILTER_SIZE * FILTER_SIZE; i++) 
		temp[i] = P[i] + R2[i];
	
	/* Invert temp by first performing an LU Decomposition */
	info = clapack_sgetrf(CblasRowMajor, mtrx_sz, mtrx_sz, temp, mtrx_sz, ipiv);

	/* Now, invert given the LU decomposition */
	info = clapack_sgetri(CblasRowMajor, mtrx_sz, temp, mtrx_sz, ipiv);

	/* W2 = P * temp */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		P, FILTER_SIZE, temp, FILTER_SIZE, 0.0f, W2, FILTER_SIZE);

	/**** 6. Update the estimate ****/

	/* W1 * residual_s1' */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, 1, 1.0f,
		W1, FILTER_SIZE, residual_s1, FILTER_SIZE, 0.0f, temp, FILTER_SIZE);

	/* W2 * residual_s2' */	
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, 1, 1.0f,
		W2, FILTER_SIZE, residual_s2, FILTER_SIZE, 0.0f, temp2, FILTER_SIZE);
	/* temp = temp + temp2 */
	for(i=0; i<FILTER_SIZE; i++)
		temp[i] = temp[i] + temp2[i];

	/* predicted = new_state + temp */
	for(i=0; i<FILTER_SIZE; i++){
		predicted[i] = new_state[i] + temp[i];
	        current_state[i] = predicted[i];
	}

	/* Clear the temp vars */
	memset(temp, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
	memset(temp2, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);

	diag(eye, 1);

	/* temp3 = eye - W1 */
	for(i=0; i<FILTER_SIZE*FILTER_SIZE; i++)
		temp3[i] = eye[i] - W1[i];
	/* temp2 = temp3 * P */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp3, FILTER_SIZE, P, FILTER_SIZE, 0.0f, temp2, FILTER_SIZE);
	/* temp = temp2 * (temp3)' */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp2, FILTER_SIZE, temp3, FILTER_SIZE, 0.0f, temp, FILTER_SIZE);

	/* temp3 = W1 * R1 */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		W1, FILTER_SIZE, R1, FILTER_SIZE, 0.0f, temp3, FILTER_SIZE);
	/* temp2 = temp2 * W1' */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp3, FILTER_SIZE, W1, FILTER_SIZE, 0.0f, temp2, FILTER_SIZE);
	/* Ptmp = temp + temp2 */
	for(i=0; i<FILTER_SIZE*FILTER_SIZE; i++)
		Ptmp[i] = temp[i] + temp2[i];	

	/* temp3 = eye - W2 */
	for(i=0; i<FILTER_SIZE*FILTER_SIZE; i++)
		temp3[i] = eye[i] - W2[i];
	/* temp2 = temp3 * P */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp3, FILTER_SIZE, P, FILTER_SIZE, 0.0f, temp2, FILTER_SIZE);
	/* temp = temp2 * (temp3)' */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp2, FILTER_SIZE, temp3, FILTER_SIZE, 0.0f, temp, FILTER_SIZE);

	/* temp3 = W2 * R2 */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		W2, FILTER_SIZE, R2, FILTER_SIZE, 0.0f, temp3, FILTER_SIZE);
	/* P = temp2 * W2' */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp3, FILTER_SIZE, W2, FILTER_SIZE, 0.0f, temp2, FILTER_SIZE);

	/* P = Ptmp + temp + temp2 */
	for(i=0; i<FILTER_SIZE*FILTER_SIZE; i++)
		P[i] = Ptmp[i] + temp[i] + temp2[i];	
	
	//	pmat(P);

	free(ipiv);
	return;
}
void Kalman::rovioKalmanFilterSetVelocity( float *velocity )
{
  // changes the velocity values in the state vector
  // velocity paramter is {vx,vy,vth}
  current_state[3] = velocity[0];
  current_state[4] = velocity[1];
  current_state[5] = velocity[2];
}

void Kalman::rovioKalmanFilterSetUncertainty( float *uncertainty )
{
  // sets the process and sensor uncertainty matrices
  // uncertainty is a nine element float vector
  // set to {proc_x,proc_y,proc_th,ns_x,ns_y,ns_th,we_x,we_y,we_th}

  
  // initialize the uncertainty array
  Q[ROWCOL(0,0)]     = uncertainty[0];
  Q[ROWCOL(1,1)]     = uncertainty[1];
  Q[ROWCOL(2,2)]     = uncertainty[2];

  R1[ROWCOL(0,0)]     = uncertainty[3];
  R1[ROWCOL(1,1)]     = uncertainty[4];
  R1[ROWCOL(2,2)]     = uncertainty[5];

  R2[ROWCOL(0,0)]     = uncertainty[6];
  R2[ROWCOL(1,1)]     = uncertainty[7];
  R2[ROWCOL(2,2)]     = uncertainty[8];
}