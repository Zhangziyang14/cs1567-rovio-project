#define PI 3.14159265

#define ANGLE_WHEEL_RIGHT 0.523598776
#define ANGLE_WHEEL_LEFT 2.61799388
#define ANGLE_WHEEL_REAR 1.57079633

#define OK 0
#define FAIL -1

#define FAILED(x) ((x) == OK) ? false : true

float thetaCorrection;

// returns average of computed left and right encoder x-axis motion
float WheelAverageX( float rightEncoder, float leftEncoder )
{
	float rightFinal, leftFinal;
	rightFinal = rightEncoder * cos( ANGLE_WHEEL_RIGHT );
	leftFinal = leftEncoder * cos( ANGLE_WHEEL_LEFT );
	return (rightFinal + leftFinal) / 2;
}

// returns average of computed left and right encoder y-axis motion
float WheelAverageY( float rightEncoder, float leftEncoder )
{
	float rightFinal, leftFinal;
	rightFinal = rightEncoder * sin( ANGLE_WHEEL_RIGHT );
	leftFinal = leftEncoder * sin( ANGLE_WHEEL_LEFT );
	return (rightFinal + leftFinal) / 2;
}

/*
DEPRECATED
float CorrectTheta( float theta )
{
	float temp, newTheta;
	
	// check if corrected value exceeds -2pi
	if ( (theta + THETA_CORRECTION) > PI )
	{
		temp = theta + THETA_CORRECTION;
		newTheta = -1 * ( temp - PI );
	}
	
	// otherwise, just correct
	else
	{
		newTheta = theta + THETA_CORRECTION;
	}
	
	return newTheta;
}
*/

// corrects theta to 0 at beginning of run
float CorrectTheta( float oldTheta )
{
	// positive theta origin case
	if ( thetaCorrection >= 0 )
	{
		if ( oldTheta - thetaCorrection < -PI )
			return -1 * ( oldTheta - thetaCorrection + PI );
		else
			return oldTheta - thetaCorrection;
	}
	
	// negative theta origin case
	else
	{
		if ( oldTheta + thetaCorrection > PI )
			return -1 * ( oldTheta + thetaCorrection - PI );
		else
			return oldTheta + thetaCorrection;
	}
}


// handle calibration of theta when changing rooms
int RoomSwitch( float oldRoomTheta, float newRoomTheta )
{
	if ( oldRoomTheta - newRoomTheta > -PI )
		thetaCorrection = oldRoomTheta - newRoomTheta;
	else
		thetaCorrection = -1 * ( oldRoomTheta - newRoomTheta + PI );
}