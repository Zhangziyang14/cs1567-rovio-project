#include <robot_if++.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>

#include <stdlib.h>
#include <signal.h>
#include <fstream>

#include "fir.h"
#include "Functions.h"
extern "C" {
	#include "kalman/kalmanFilterDef.h"
}

#define FILEDUMP

#define DEBUG
#define FAILED(x) ((x) == OK) ? false : true

/******define CONSTANTS******/
#define ACTION_TURN 0x101/*action flag pass to update function*/
#define ACTION_MOVE 0x102/*action flag pass to update function*/
#define OK 0
#define FAIL -1

/******define GLOBALS******/
float *predicted;

int currentRoom;
float currNSX = 0, currNSY = 0, currNSTheta = 0;
float prevNSX = 0, prevNSY = 0, prevNSTheta = 0;

float prevWX = 0, prevWY = 0, prevWT = 0;
float currWX = 0, currWY = 0, currWT = 0;
float originDegree;

float roomDegree;
float roomTheta;

filter *xFilter = NULL;
filter *yFilter = NULL;
filter *rightFilter = NULL;
filter *leftFilter = NULL;
filter *rearFilter = NULL;
kalmanFilter *kf = NULL;
RobotInterface *robot;


#ifdef FILEDUMP
std::ofstream dataFile;
#endif

	
// initialize filters and prime coord filters w/ 3 readings
int InitializeFirFilters( RobotInterface *robot )
{
	SIGNAL_CHECK

	// initialize filters
	xFilter = FirFilterCreate();
	yFilter = FirFilterCreate();
	rightFilter = FirFilterCreate();
	leftFilter = FirFilterCreate();
	rearFilter = FirFilterCreate();
	
	// prime filters w/ 3 readings
	for ( int i=0; i<3; i++ )
	{
		robot->update();
		FirFilter( xFilter, robot->X() );
		FirFilter( yFilter, robot->Y() );
	}
	
	return OK;
}

// initializes kalman filter to default values
int InitializeKalmanFilter( RobotInterface *robot )
{
	float *initPose = (float *)calloc(3, sizeof(float));
	float *velocity = (float *)calloc(3, sizeof(float));
	predicted = (float *)calloc(9, sizeof(float));
	kf = (kalmanFilter *)malloc(sizeof(kalmanFilter));
	
	float firX, firY;
	
	int deltat = .1;
	float thetaSum = 0.0;

	if ( initPose == NULL || velocity == NULL || predicted == NULL || kf == NULL )
	{
		printf("Allocation failure, exiting\n");
		exit(-1);
	}
	
	SIGNAL_CHECK
	
	firX = FirFilter(xFilter, robot->X());
	firY = FirFilter(yFilter, robot->Y());
	
	initPose[0] = CorrectXtoCM( firX, robot->RoomID() );
	initPose[1] = CorrectYtoCM( firY, robot->RoomID() );
	
	for ( int i=0; i<10; i++ )
	{
		robot->update();
		thetaSum += CorrectTheta( robot->Theta(), robot->RoomID() );
	}
	
	initPose[2] = thetaSum / 10;
	
	velocity[0] = 0.0;
	velocity[1] = 0.0;
	velocity[2] = 0.0;
	
	#ifdef DEBUG
		printf( "Initial Pose:\nX: %.3f Y: %.3f Theta(Rad): %.3f Theta(Deg): %.3f\n", initPose[0], initPose[1], initPose[2], TODEGREE(initPose[2]) );
	#endif
	
	initKalmanFilter( kf, initPose, velocity, deltat );
	
	free(initPose);
	free(velocity);
	
	return OK;
}

// updates kalman filter with current sensor data, returns prediction array
float *UpdateKalman( )
{
	float *curPose = (float *)calloc(3, sizeof(float));
	float *curWheelEnc = (float *)calloc(3, sizeof(float));
	
	float rightEncData, leftEncData, rearEncData, yMovement, xEncCoord, yEncCoord, encTheta;
	
	// correct NS data to cm-based coordinates, flip x and y for odd-numbered rooms
	if ( robot->RoomID() % 2 == 0 )
	{
		curPose[0] = CorrectXtoCM( FirFilter(xFilter, robot->X()), robot->RoomID() );	//NS X
		curPose[1] = CorrectYtoCM( FirFilter(yFilter, robot->Y()), robot->RoomID() );	//NS Y
	}
	else
	{
		curPose[0] = CorrectYtoCM( FirFilter(yFilter, robot->Y()), robot->RoomID() );	//NS X
		curPose[1] = CorrectXtoCM( FirFilter(xFilter, robot->X()), robot->RoomID() );	//NS Y
	}
	curPose[2] = CorrectTheta( robot->Theta(), robot->RoomID() );					//NS Theta
	
	// get wheel encoder data for this movement
	rightEncData = FirFilter( rightFilter, robot->getWheelEncoder( RI_WHEEL_RIGHT) );
	leftEncData = FirFilter( leftFilter, robot->getWheelEncoder( RI_WHEEL_LEFT ) );
	rearEncData = FirFilter( rearFilter, robot->getWheelEncoder( RI_WHEEL_REAR ) );
	yMovement = WheelAverageY( rearEncData, leftEncData ) / TICKS_PER_CM;
	
	// use previous kalman filter values to determine new x y position
	xEncCoord = predicted[0] + (yMovement * cos(predicted[2]));
	yEncCoord = predicted[1] + (yMovement * sin(predicted[2]));
	//encTheta = predicted[2] + ( ( rearEncData / 5.5 ) / TICKS_PER_CM );
	encTheta = curPose[2];
	//printf("REARENCDATA: %.3f ENCTHETA: %.3f PREDICTED[2]: %.3f\n", rearEncData, encTheta, predicted[2]);
	
	curWheelEnc[0] = xEncCoord; 		// Wheel Encoder X position
	curWheelEnc[1] = yEncCoord; 		// Wheel Encoder Y position
	curWheelEnc[2] = encTheta;			// Wheel Encoder Theta
	
	rovioKalmanFilter( kf, curPose, curWheelEnc, predicted );
	
	#ifdef DEBUG
	printf("NorthStar: X: %.4f Y: %.3f Theta(Rad): %.3f Theta(Deg): %.3f\n", curPose[0], curPose[1], curPose[2], TODEGREE(curPose[2]) );
	printf("Wheel Encoder: X: %.3f Y: %.3f Theta(Rad): %.3f Theta(Deg): %.3f\n", curWheelEnc[0], curWheelEnc[1], curWheelEnc[2], TODEGREE(curWheelEnc[2]) );
	printf("Predicted: X: %.3f Y: %.3f Theta(Rad): %.3f Theta(Deg): %.3f\n", predicted[0], predicted[1], predicted[2], TODEGREE(predicted[2]));
	#endif
	
	#ifdef FILEDUMP
	dataFile << robot->RoomID() <<" "<< curPose[0] <<" "<< curPose[1] <<" "<< TODEGREE(curPose[2]) <<"\n"; 
	#endif
	
	free(curPose);
	free(curWheelEnc);
	
	return predicted;
}

/* DEPRECATED
void updateNorthStar(int flag,int roomID)
{
	
	//update NS for TURN
	if(flag==ACTION_TURN){
		int step = 0;
		float degree = 0;
		float rawD = 0;
		float myTheta=0;
		do{
			while(robot->update() != RI_RESP_SUCCESS) {
				printf("Failed to update sensor information!\n");
			}
			if(step>13){
				degree += CorrectTheta(robot->Theta(), robot->RoomID());
				rawD += TODEGREE(robot->Theta());
				myTheta += TORADIAN(CorrectTheta(robot->Theta(), robot->RoomID()));
			}
			step++;
			
		}
		while(step<20);

		roomDegree = degree/6;
		roomTheta = myTheta/6;

		printf("@@myTheta:%4.2f\n",roomTheta);

	}
	//update NS for MOVE
	else if(flag==ACTION_MOVE){
		if(ACTION_MOVE == flag) {//it's a move update
		}

	}

	//printf("updataNS: degree: %4.2f raw:%4.2f\n",TODEGREE(predicted[2]),rawD/10);

}

void updateWheelEncoder(int flag, float currentTheta)
{
	if(flag == ACTION_MOVE) {//it's a move update
		float theta = currentTheta;
		float coe = 0.40; //cm per unit 

		float filterR =  FirFilter(rightFilter,robot->getWheelEncoder(RI_WHEEL_RIGHT));
		float filterL =   FirFilter(leftFilter,robot->getWheelEncoder(RI_WHEEL_LEFT));
		float filterB =   FirFilter(rearFilter,robot->getWheelEncoder(RI_WHEEL_REAR));

		float moveX = wheelMovedX(filterR, filterL) * coe;//get x contribution after filter  
		float moveY = wheelMovedY(filterR, filterL, filterB) * coe;//get y contribution after filter
		//printf("--fr %5.2f, fl %5.2f, fb %5.2f degree:%4.2f\n",filterR,filterL,filterB,TODEGREE(theta));
		//printf("---moveX:%5.2f, moveY:%5.2f\n", moveX, moveY);
		prevWX = currWX;
		prevWY = currWY;
		moveX=0;
		currWX += -sin(theta)*moveY+cos(theta)*moveX;
		//currWY += cos(theta)*moveY-sin(theta)*moveX;
		currWY += cos(theta)*moveY+sin(theta)*moveX;


		printf("currWX:%4.2f, currWY:%4.2f\n",currWX,currWY);

	}
	else if(flag == ACTION_TURN){
		float filterB =   FirFilter(rearFilter,robot->getWheelEncoder(RI_WHEEL_REAR));
		float coeTurnWheel = 2.1159924;//wheel encoder coefficient turn tick into degrees
		float deltaTheta = coeTurnWheel*filterB;	//calcualte estimate 
		currWT += deltaTheta;

		printf("currWT:%4.2f\n",currWX,currWY);
	}

}
*/

int TurnTo(float target, float margin)
{
	float thetaCurrent, thetaSum;
	float offset = margin;
	float diff;//diff between current and target
	//PID *pid = new PID();
	bool multiple_sample = true;//indicate if we need to take multiple sample
	int turn_speed = 8;
	int turn_flag = RI_TURN_LEFT;//set initial turn flag to turn left
	
	float *vel = (float *)calloc(3, sizeof(float));
	
	do {
		thetaCurrent = TODEGREE(predicted[2]);
		diff = MINABS(target-thetaCurrent, 360-thetaCurrent+target);//get diff between target and current
		
		printf("-1target:%4.2f curr:%4.2f diff:%4.2F\n",target,thetaCurrent,diff);

		if(ABS(diff)<=offset)//break loop condition if diff is within a preset acceptable offset
			break;

		//if absolute value of diff >= 20
		if(ABS(diff) >= 20) {
			turn_flag = diff>0? RI_TURN_RIGHT_20DEG: RI_TURN_LEFT_20DEG;//set turn flag
			turn_speed = 4;

			if(ABS(diff)>=40)
				multiple_sample = false;//diff is greater than 40, then we need more 20degree turn
			else
				multiple_sample = true;//diff between 20 and 40, we need one more 20degree turn

			robot -> Move(turn_flag, turn_speed);
			diff += (diff>0?-1:1)*20;//update diff
			
			// update kalman
			vel[2] = turn_speed;
			rovioKalmanFilterSetVelocity( kf, vel );
			robot->update();
			UpdateKalman();
		}
		//absolute value of diff < 20
		//doing adjustment
		else{
			multiple_sample = true;
			turn_flag = diff>0? RI_TURN_RIGHT: RI_TURN_LEFT;
			turn_speed = 6;

			//update sensor
			SIGNAL_CHECK

			robot -> Move(turn_flag, turn_speed);

			// update kalman
			vel[2] = turn_speed;
			rovioKalmanFilterSetVelocity( kf, vel );
			robot->update();
			UpdateKalman();
			
			SIGNAL_CHECK
			
			if(!robot->IR_Detected()) {
				robot -> Move(RI_STOP, turn_speed);
			}

		}		
		
		printf("target:%4.2f curr:%4.2f diff:%4.2F\n\n",target,thetaCurrent,diff);
	}
	while(1);
	
	// reset kalman velocity
	vel[2] = 0;
	rovioKalmanFilterSetVelocity( kf, vel );
	robot->update();
	UpdateKalman();
	
	free(vel);

	return OK;
}

/* DEPRECATED
void MoveToXY(float targetX, float targetY, float margin){
	static int runnum = 1;    
	//PID *pid = new PID();	
	bool RoomChanged = false;    
	float XYRange = sqrt(margin); 
	float Xoffset=0;
	float Yoffset=0;
	float thetaOffset = TORADIAN(3);
	bool NewTarget = false;    
	float theta = predicted[2];
	float disTravel;
	float disoffset;
	
	//total distance we need to travel
	float totalDistance = sqrt(pow(targetX-predicted[0],2)+
						  pow(targetY-predicted[1],2));

	float beginX = predicted[0], beginY = predicted[1];
	int move_flag = RI_MOVE_FORWARD;
	int move_speed = 1;//initial speed set to 1

	do{
		while(robot->update() != RI_RESP_SUCCESS) {
			printf("Failed to update sensor information!\n");
		}
		// TODO handle room change
		printf("**aveTheta:%4.2f MoveTo\n",predicted[2]);

		updateWheelEncoder(ACTION_MOVE,theta);
		updateWheelEncoder(ACTION_TURN,theta);

		Xoffset = targetX - predicted[0];
		Yoffset = targetY - predicted[1];
		   
		disTravel = sqrt(pow(beginX-predicted[0],2)+
						  pow(beginY-predicted[1],2));//distance we travel so far

		disoffset = (sqrt(pow(Xoffset,2)+pow(Yoffset,2)));//how far we still need to travel
		
		if(disTravel>totalDistance)//if travel distance > total distance then disoffset becomes negative
			disoffset = (-1)*disoffset;

		printf(" offset: %6.2f, ",disoffset);

		//if distance offset is less than margin, done
		if(disoffset <= margin) {
			if(!robot->IR_Detected()){//issue stop command
				robot->Move(RI_STOP,move_speed);
			}
			
			NewTarget = true;//we dont need to check new target
			break;
		}

		//we already reach targetX or targetY
		if((ABS(Xoffset)<=XYRange || ABS(Yoffset)<=XYRange) && NewTarget==false) {
			printf("\n-----------------check need for  new destination---------------\n");
			NewTarget = true;            //we already check it before

			if(ABS(Xoffset)<=XYRange) {    //we get to targeX, but not target Y
				if(Yoffset>0) {                    //current Y is too large, then we should face 270 degree
					if(ABS(predicted[2]-TORADIAN(0))>thetaOffset){//if we are not facing 270 degree
						printf("----target %5.2f, %5.2f--------Im at %5.2f,%5.2f, need to face 270\n",
							   targetX,targetY,predicted[0],predicted[1]);
						TurnTo(0.0);
						MoveToXY(targetX,targetY,margin);//move to a new destination
						break;
					}
				}
				else{
					if(ABS(predicted[2]-TORADIAN(180))>thetaOffset){//if we are not facing 270 degree
						printf("----target %5.2f, %5.2f--------Im at %5.2f,%5.2f, need to face 270\n",
							   targetX,targetY,predicted[0],predicted[1]);
						TurnTo(180.0);
						MoveToXY(targetX,targetY,margin);//move to a new destination
						break;
					}

				}
			}
			else if((ABS(Yoffset)<=XYRange)){
				if(Xoffset>0) {  
					if(ABS(predicted[2]-TORADIAN(270))>thetaOffset){//if we are not facing 270 degree
						printf("----target %5.2f, %5.2f--------Im at %5.2f,%5.2f, need to face 270\n",
							   targetX,targetY,predicted[0],predicted[1]);
						TurnTo((float)270.0);
						MoveToXY(targetX,targetY,margin);//move to a new destination
						break;
					}

				}
				else{
					if(ABS(predicted[2]-TORADIAN(90))>thetaOffset){//if we are not facing 270 degree
						printf("----target %5.2f, %5.2f--------Im at %5.2f,%5.2f, need to face 270\n",
							   targetX,targetY,predicted[0],predicted[1]);
						TurnTo((float)90.0);
						MoveToXY(targetX,targetY,margin);//move to a new destination
						break;
					}
				}

			}


		}

		if(disoffset<0) {
			move_flag = RI_MOVE_BACKWARD;
		}
		else {
			move_flag = RI_MOVE_FORWARD;
		}

		if(!robot->IR_Detected()) {//do the move 
			robot -> Move(move_flag, move_speed);
			theta = predicted[2];
		}


	}while(1);
}
*/

void MoveTo( float targetX, float targetY )
{
	float targetTheta, targetDist;
	float prevX, prevY;
	int speed;
	
	float *vel = (float *)calloc(3, sizeof(float));
	
	//get intial kalman prediction
	UpdateKalman();
	
	//determine proper angle for path
	targetTheta = atan2( (targetY - predicted[1]), (targetX - predicted[0]) );
	targetDist = GetDistance(predicted[0], targetX, predicted[1], targetY);
	
	printf("Traveling %.3fcm using theta %.3f\n", targetDist, TODEGREE(targetTheta));
	TurnTo( TODEGREE(targetTheta), 5 );
	
	printf("Moving!\n");
	while(currDist <= targetDist)
	{
		// get x & y before move
		prevX = predicted[0];
		prevY = predicted[1];
		
		robot->Move( RI_MOVE_FORWARD, speed );
		
		// update kalman
		vel[1] = speed;
		rovioKalmanFilterSetVelocity( kf, vel );
		robot->update();
		UpdateKalman();
		
		// update current distance
		currDist += GetDistance(prevX, predicted[0], prevY, predicted[1]);
		
		printf("currDist: %.3f target: %.3f\n", currDist, targetDist);
	}
	
	// reset kalman velocity
	vel[1] = 0;
	rovioKalmanFilterSetVelocity( kf, vel );
	robot->update();
	UpdateKalman();
	
	free(vel);
}

// handle ctrl + c
void QuitHandler( int s )
{
	printf( "Caught Ctrl + C. Dying a (more) graceful death\n" );
	
	delete(robot);
	
	#ifdef FILEDUMP
	dataFile.close();
	#endif
	
	exit(-1);
}

int main(int argv, char **argc)
{
	// set up ctrl+c handler struct
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = QuitHandler;
	sigemptyset( &sigIntHandler.sa_mask );
	sigIntHandler.sa_flags = 0;
	sigaction( SIGINT, &sigIntHandler, NULL );

	// Make sure we have a valid command line argument
	if(argv <= 1) {
		std::cout << "Usage: robot_test <address of robot>" << std::endl;
		exit(-1);
	}

	// Setup the robot interface
	robot = new RobotInterface(argc[1],0);
	
	InitializeFirFilters( robot );
	InitializeKalmanFilter( robot );
	
	#ifdef FILEDUMP
	dataFile.open("data/data.txt");
	#endif
	
	MoveTo( -20, 158 );
	
	// Clean up
	delete(robot);
	
	#ifdef FILEDUMP
	dataFile.close();
	#endif
	
	return 0;
}