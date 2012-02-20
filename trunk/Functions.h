/**
 * Function.h
 * utility function
 */

#define COS_R 0.8660254            // cos(30).
#define COS_L -0.8660254		   // cos(150).
#define SIN 0.5                    // sin(30) and sin(150).
#define PI 3.1415926
#define DIAMETER 29
#define TODEGREE(x) ((float)((x)*57.29578)) //turn theta into degree
#define ABS(x) ((x)>0?(x):-(x))
#define TORADIAN(x) ((float)(x*0.0174533)) //turn degree to radian
#define MINABS(x,y) (ABS(x)<ABS(y)?(x):(y))//return the one with lower absolute value

 


float wheelMovedY(float rightWheel, float leftWheel, float rearWheel) {
	float x;
	if(rightWheel!=0 && leftWheel!=0 && rearWheel==0)//forward
		x = ((rightWheel * SIN) + (leftWheel * SIN))/2;
	else if(rightWheel!=0 && leftWheel==0) //
		x = ((rightWheel * SIN) + rearWheel)/2;
	else if(rightWheel==0 && leftWheel!=0)
		x = ((leftWheel * SIN) + rearWheel)/2;
	else
		x = rearWheel;
	return x;
}

float wheelMovedX(float rightWheel, float leftWheel) {
        float y;
 
        if (rightWheel > 0 && leftWheel > 0) {
           y = ((rightWheel * COS_R) + (leftWheel * COS_L)) / 2;
		   //printf("wheelMoveX: %4.2f\n",y);
        }
        else if (rightWheel == 0 && leftWheel > 0) {
           y = leftWheel * COS_L;
        }
        else if (rightWheel > 0 && leftWheel == 0) {
           y = rightWheel * COS_R;
        }
        else {
           y = 0;
		   //printf("000wheelMoveX\n");
        }
 
        return y;
}


 
 
/**
 *
 */
float wheelMovedTheta(int rearWheel, float curWheelTheta) {
        float t;
 
        t = curWheelTheta + (rearWheel / (PI * DIAMETER));
 
        return t;
}


float NSDegreeToMyDegree(float oldD, int roomID){
	float degree = oldD;
	switch(roomID) {
		case 2: //room 2
			degree = 360+75 - oldD;
			break;
		case 3: //room3
			degree = 360 - oldD;
			break;
		case 4: //room 4
			degree = 360+55 - oldD;
			break;
		case 5: //room 5
			degree = 360+1 - oldD;
			break;
		default:
			//printf("room %d, Man, are you sure you are not in outer space!\n",roomID);
			return 0;
	}
	return (float)(((int)degree)%360);
}
