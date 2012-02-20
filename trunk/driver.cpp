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

//#define FILEDUMP
//#define DUMP_NS




#define FAILED(x) ((x) == OK) ? false : true

/******define CONSTANTS******/
#define ANGLE_WHEEL_RIGHT 0.523598776
#define ANGLE_WHEEL_LEFT 2.61799388
#define ANGLE_WHEEL_REAR 1.57079633
#define ACTION_TURN 0x101/*action flag pass to update function*/
#define ACTION_MOVE 0x102/*action flag pass to update function*/
#define TICKS_PER_INCH 5.2
#define NS_PER_INCH 124
#define NS_X_ORIGIN -13159.6
#define NS_Y_ORIGIN -1355.9
#define CORRECT_NS_X(x) (x - NS_X_ORIGIN) / NS_PER_INCH
#define CORRECT_NS_Y(x) (x - NS_Y_ORIGIN) / NS_PER_INCH
#define OK 0
#define FAIL -1

/******define GLOBALS******/
float xNSOrigin, yNSOrigin;
int xEncCoord, yEncCoord;
float yTotal;
float thetaCorrection;
int currentRoom;
float aveWeightedDegree;
float aveWeightedTheta;
float aveWeightedX;
float aveWeightedY;
float currNSX,currNSY, currNSTheta;
float prevNSX, prevNSY, prevNSTheta;

float prevWX, prevWY, prevWT;
float currWX, currWY, currWT;
float originDegree;

float roomDegree;
float roomTheta;

filter *xFilter = NULL;
filter *yFilter = NULL;
filter *rightWheelFilter = NULL;
filter *leftWheelFilter = NULL;
filter *rearWheelFilter = NULL;
kalmanFilter *kf = NULL;
RobotInterface *robot;


#ifdef FILEDUMP
std::ofstream rawDataFile;
std::ofstream filterDataFile;
#endif

// returns average of computed left and right encoder x-axis motion
float WheelAverageX( float rightEncoder, float leftEncoder )
{
	float rightFinal, leftFinal;
	rightFinal = rightEncoder * cos( ANGLE_WHEEL_RIGHT );
	leftFinal = leftEncoder * cos( ANGLE_WHEEL_LEFT );
	//printf( "WheelAverageX():\nrightFinal: %.3f leftFinal: %.3f\n", rightFinal, leftFinal );
	return (rightFinal + leftFinal) / 2;
}

// returns average of computed left and right encoder y-axis motion
float WheelAverageY( float rightEncoder, float leftEncoder )
{
	float rightFinal, leftFinal;
	rightFinal = rightEncoder * sin( ANGLE_WHEEL_RIGHT );
	leftFinal = leftEncoder * sin( ANGLE_WHEEL_LEFT );
	//printf( "WheelAverageY():\nrightFinal: %.3f leftFinal: %.3f\n", rightFinal, leftFinal );
	return (rightFinal + leftFinal) / 2;
}





void measurement(){
	long sumd = 0;
	long sumX = 0;
	long sumY = 0;
	int step = 1;

	int roomID = robot->RoomID();
	int degree = NSDegreeToMyDegree(robot->Theta(),roomID);
	int x = FirFilter(xFilter,robot->X());
	int y = FirFilter(yFilter,robot->Y());;


	while(1){
		while(robot->update() != RI_RESP_SUCCESS) {
				printf("Failed to update sensor information!\n");
		}
		
		roomID = robot->RoomID();
		degree = NSDegreeToMyDegree(robot->Theta(),roomID);
		x = FirFilter(xFilter,robot->X());
		y = FirFilter(yFilter,robot->Y());

		sumd += degree;
		sumX += x;
		sumY += y;
		printf("X: %5d, Y: %5d, degree: %4d, avg: %6d|%6d|%4d, signal %d|%6d\n",
			   x,y,degree, sumX/step,sumY/step,sumd/step, roomID,robot->NavStrengthRaw());
		//X:  -313, Y:  3180, degree:   77, avg:   -292-  3190-  77, signal 2- 11855
		step++;

		//printf("updataNS: degree: %4.2f raw:%4.2f\n",aveWeightedDegree,rawD/10);
	}

}



	
// initialize filters and prime coord filters w/ 3 readings
int InitializeFirFilters( RobotInterface *robot )
{
	if ( robot->update() == RI_RESP_SUCCESS ) {		
		// initialize filters
		xFilter = FirFilterCreate();
		yFilter = FirFilterCreate();
		rightWheelFilter = FirFilterCreate();
		leftWheelFilter = FirFilterCreate();
		rearWheelFilter = FirFilterCreate();
		
		// prime filters w/ 3 readings
		for ( int i=0; i<3; i++ )
		{
			robot->update();
			FirFilter( xFilter, CORRECT_NS_X(robot->X()) );
			FirFilter( yFilter, CORRECT_NS_Y(robot->Y()) );
		}
		
		return OK;
	}
	
	return FAIL;
}
// initializes kalman filter to default values
// IMPORTANT NOTE: NS X is same direction as Encoder Y, so NS X AND Y ARE REVERSED IN THE ARRAY
int InitializeKalmanFilter( RobotInterface *robot )
{
	float *initPose = (float *)malloc(sizeof(float) * 3);
	float *velocity = (float *)malloc(sizeof(float) * 3);
	int deltat = 1;
	
	float thetaSum = 0.0;
	
	kf = (kalmanFilter *)malloc(sizeof(kalmanFilter));

	if ( robot->update() == RI_RESP_SUCCESS )
	{
		initPose[0] = FirFilter( yFilter, CORRECT_NS_Y(robot->Y()) );
		initPose[1] = FirFilter( xFilter, CORRECT_NS_X(robot->X()) );
		
		for ( int i=0; i<10; i++ )
		{
			robot->update();
			thetaSum += TODEGREE( robot->Theta() );
		}
		
		initPose[2] = thetaSum / 10;
		
		velocity[0] = 0.0;
		velocity[1] = 0.0;
		velocity[2] = 0.0;
		
		initKalmanFilter( kf, initPose, velocity, deltat );
		
		return OK;
	}
	
	return FAIL;
}

// updates kalman filter with current sensor data, returns prediction array
float *UpdateKalman()
{
	float *curPose = (float *)malloc(sizeof(float) * 3);
	float *curWheelEnc = (float *)malloc(sizeof(float) * 3);
	float *predicted = (float *)malloc(sizeof(float) * 9);
	
	float x, y, theta;
	
	curPose[0] = FirFilter( yFilter, CORRECT_NS_Y(robot->Y()) );
	curPose[1] = FirFilter( xFilter, CORRECT_NS_X(robot->X()) );
	curPose[2] = TODEGREE( robot->Theta() );
	
	printf("X: %.4f Y: %.3f\n", CORRECT_NS_X((float) robot->X()), CORRECT_NS_Y((float) robot->Y()));
	
	x = WheelAverageX( robot->getWheelEncoder( RI_WHEEL_RIGHT), robot->getWheelEncoder( RI_WHEEL_LEFT ) ) / TICKS_PER_INCH;
	y =  WheelAverageY( robot->getWheelEncoder( RI_WHEEL_RIGHT), robot->getWheelEncoder( RI_WHEEL_LEFT ) ) / TICKS_PER_INCH;
	theta = robot->getWheelEncoder( RI_WHEEL_REAR ) / ( PI * 11 ) / TICKS_PER_INCH;
	
	curWheelEnc[0] = x; // X
	curWheelEnc[1] = y; // Y
	curWheelEnc[2] = theta; // Theta
	
	rovioKalmanFilter( kf, curPose, curWheelEnc, predicted );
	
	return predicted;
}

// determines approx origin by averaging 10 readings
int SetOrigin()
{
	float xSum = 0;
	float ySum = 0;
	float thetaSum = 0;

	for ( int i=0; i<10; i++)
	{
		robot->update();
		xSum += FirFilter( xFilter, CORRECT_NS_X(robot->X()) );
		ySum += FirFilter( yFilter, CORRECT_NS_Y(robot->Y()) );
	}
	
	xNSOrigin = xSum /10;
	yNSOrigin = ySum /10;
	
	/*
	originDegree = 0;
	for(int i=0;i<10;i++){
		updateNorthStar(ACTION_TURN,1);
		originDegree += aveWeightedDegree;
	}
	originDegree = originDegree/10;//setting origin degree
	*/

	xEncCoord = 0;
	yEncCoord = 0;



	currWX = 0; currWY = 0; currWT = 0;
	prevWX = 0; prevWY = 0; prevWT = 0;

	return OK;
}

void calculateWeightedAve(int flag){
	if(flag==ACTION_MOVE){
		aveWeightedX = currWX;
		aveWeightedY = currWY;


	}else if(flag==ACTION_TURN){
		aveWeightedTheta = roomTheta;
		aveWeightedDegree = roomDegree;
	}
}

void updateNorthStar(int flag,int roomID){
	
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
				degree += NSDegreeToMyDegree(TODEGREE(robot->Theta()), robot->RoomID());
				rawD += TODEGREE(robot->Theta());
				myTheta += TORADIAN(NSDegreeToMyDegree(TODEGREE(robot->Theta()), robot->RoomID()));
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



	//printf("updataNS: degree: %4.2f raw:%4.2f\n",aveWeightedDegree,rawD/10);

}

void updateWheelEncoder(int flag, float currentTheta){
	if(flag == ACTION_MOVE) {//it's a move update
		float theta = currentTheta;
		float coe = 0.40; //cm per unit 

		float filterR =  FirFilter(rightWheelFilter,robot->getWheelEncoder(RI_WHEEL_RIGHT));
		float filterL =   FirFilter(leftWheelFilter,robot->getWheelEncoder(RI_WHEEL_LEFT));
		float filterB =   FirFilter(rearWheelFilter,robot->getWheelEncoder(RI_WHEEL_REAR));

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
		float filterB =   FirFilter(rearWheelFilter,robot->getWheelEncoder(RI_WHEEL_REAR));
		float coeTurnWheel = 2.1159924;//wheel encoder coefficient turn tick into degrees
		float deltaTheta = coeTurnWheel*filterB;	//calcualte estimate 
		currWT += deltaTheta;

		printf("currWT:%4.2f\n",currWX,currWY);
	}

}



int TurnTo2(float target, float margin)
{
	float thetaCurrent, thetaSum;
	float offset = margin;
	float diff;//diff between current and target
	//PID *pid = new PID();
	bool multiple_sample = true;//indicate if we need to take multiple sample
	int turn_speed = 8;
	int turn_flag = RI_TURN_LEFT;//set initial turn flag to turn left
	
	do {

		//thetaCurrent =  aveWeightedDegree;
		if(multiple_sample){//taking samples of current theta
			updateNorthStar(ACTION_TURN,1);
			updateWheelEncoder(ACTION_TURN,aveWeightedTheta);
			calculateWeightedAve(ACTION_TURN);
			thetaCurrent =  aveWeightedDegree;
			diff = MINABS(target-thetaCurrent, 360-thetaCurrent+target);//get diff between target and current

		}
		
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
		}
		//absolute value of diff < 20
		//doing adjustment
		else{
			multiple_sample = true;
			turn_flag = diff>0? RI_TURN_RIGHT: RI_TURN_LEFT;
			turn_speed = 6;

			//update sensor
			while(robot->update() != RI_RESP_SUCCESS) {
				printf("Failed to update sensor information!\n");
			}

			robot -> Move(turn_flag, turn_speed);

			while(robot->update() != RI_RESP_SUCCESS) {
				printf("Failed to update sensor information!\n");
			}
			
			if(!robot->IR_Detected()) {
				robot -> Move(RI_STOP, turn_speed);
			}

		}
		printf("target:%4.2f curr:%4.2f diff:%4.2F\n",target,thetaCurrent,diff);
		

	}
	while(1);

	return OK;
}

int TurnTo(float target)
{
	TurnTo2(target,3);
}

void MoveToXY(float targetX, float targetY, float margin){
	static int runnum = 1;    
	//PID *pid = new PID();	
	bool RoomChanged = false;    
	float XYRange = sqrt(margin); 
	float Xoffset=0;
	float Yoffset=0;
	float thetaOffset = TORADIAN(3);
	bool NewTarget = false;    
	float theta = aveWeightedTheta;
	float disTravel;
	float disoffset;
	
	//total distance we need to travel
	float totalDistance = sqrt(pow(targetX-aveWeightedX,2)+
						  pow(targetY-aveWeightedY,2));

	float beginX = aveWeightedX, beginY = aveWeightedY;
	int move_flag = RI_MOVE_FORWARD;
	int move_speed = 1;//initial speed set to 1

	do{
		while(robot->update() != RI_RESP_SUCCESS) {
			printf("Failed to update sensor information!\n");
		}
		// TODO handle room change
		printf("**aveTheta:%4.2f MoveTo\n",aveWeightedTheta);


		updateWheelEncoder(ACTION_MOVE,theta);	
		calculateWeightedAve(ACTION_MOVE);
		updateWheelEncoder(ACTION_TURN,theta);	
		calculateWeightedAve(ACTION_TURN);



		Xoffset = targetX - aveWeightedX;
		Yoffset = targetY - aveWeightedY;
		   
		disTravel = sqrt(pow(beginX-aveWeightedX,2)+
						  pow(beginY-aveWeightedY,2));//distance we travel so far

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
					if(ABS(aveWeightedTheta-TORADIAN(0))>thetaOffset){//if we are not facing 270 degree
						printf("----target %5.2f, %5.2f--------Im at %5.2f,%5.2f, need to face 270\n",
							   targetX,targetY,aveWeightedX,aveWeightedY);
						TurnTo(0.0);
						MoveToXY(targetX,targetY,margin);//move to a new destination
						break;
					}
				}
				else{
					if(ABS(aveWeightedTheta-TORADIAN(180))>thetaOffset){//if we are not facing 270 degree
						printf("----target %5.2f, %5.2f--------Im at %5.2f,%5.2f, need to face 270\n",
							   targetX,targetY,aveWeightedX,aveWeightedY);
						TurnTo(180.0);
						MoveToXY(targetX,targetY,margin);//move to a new destination
						break;
					}

				}
			}
			else if((ABS(Yoffset)<=XYRange)){
				if(Xoffset>0) {  
					if(ABS(aveWeightedTheta-TORADIAN(270))>thetaOffset){//if we are not facing 270 degree
						printf("----target %5.2f, %5.2f--------Im at %5.2f,%5.2f, need to face 270\n",
							   targetX,targetY,aveWeightedX,aveWeightedY);
						TurnTo((float)270.0);
						MoveToXY(targetX,targetY,margin);//move to a new destination
						break;
					}

				}
				else{
					if(ABS(aveWeightedTheta-TORADIAN(90))>thetaOffset){//if we are not facing 270 degree
						printf("----target %5.2f, %5.2f--------Im at %5.2f,%5.2f, need to face 270\n",
							   targetX,targetY,aveWeightedX,aveWeightedY);
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
			theta = aveWeightedTheta;
		}


	}while(1);
}

int MoveTo2( int x, int y, float targetDegree)
{
	yTotal = 0.0;
	int speed = RI_FASTEST;
	float rightEnc, leftEnc, yTicks, xTicks, ticks, newTheta;
	bool speedCheck = true;
	float yEachLoop = 0;
	int loop = 0;
	int loopDistance = 20;
	float degreeMargin = 7;
	float xTotal = 0.0;
	
	
	// convert x&y to ticks
	yTicks = y * TICKS_PER_INCH;
	xTicks = x * TICKS_PER_INCH;
	
	ticks = sqrt( pow(xTicks, 2) + pow(yTicks, 2) );
	
	
	// move at top speed until w/in 20% of goal
	while ( 1 && !robot->IR_Detected() )
	{
		robot->Move( RI_MOVE_FORWARD, speed );
		
		while(robot->update() != RI_RESP_SUCCESS) {
				printf("Failed to update sensor information!\n");
			}
		updateWheelEncoder(ACTION_MOVE,TORADIAN(NSDegreeToMyDegree(TODEGREE(robot->Theta()),2)));
		

		yTotal =  currWY;
		xTotal =  currWX;
		if ( yTotal >= (.8 * ticks ) && speedCheck )
		{
			speed = RI_SLOWEST;
			speedCheck = false;
		}
		
		yEachLoop = (int)yTotal%loopDistance;
		if(yTotal/loopDistance>loop){
			updateNorthStar(ACTION_TURN,1);
			
			updateWheelEncoder(ACTION_MOVE,TORADIAN(aveWeightedDegree));
			/*if(ABS(aveWeightedDegree-targetDegree)>degreeMargin){
				printf("Turing to target: %4.2f\n",targetDegree);
				TurnTo(targetDegree);
				
			}//*/
			
			loop++;
		}

		//printf("ytotal:%4.2f yEach:%4.2f loop:%d\n",yTotal,yEachLoop,loop);


		if ( yTotal >= yEncCoord )
			break;
		if ( xTotal >= xEncCoord )
			break;

	}

	return OK;
}

int MoveTo(int x, int y)
{
	yTotal = 0.0;
	int speed = RI_FASTEST;
	float rightEnc, leftEnc, yTicks, xTicks, ticks, newTheta;
	bool speedCheck = true;
	
	//float *predicted = (float *)malloc(sizeof(float) * 9);
	
	y = y - yEncCoord;
	x = x - xEncCoord;
	
	// convert x&y to ticks
	yTicks = y * TICKS_PER_INCH;
	xTicks = x * TICKS_PER_INCH;
	
	ticks = sqrt( pow(xTicks, 2) + pow(yTicks, 2) );
	
	robot->update();
	
	// move at top speed until w/in 20% of goal
	while ( 1 && !robot->IR_Detected() )
	{
		robot->Move( RI_MOVE_FORWARD, speed );
		robot->update();
		
		rightEnc = FirFilter( rightWheelFilter, robot->getWheelEncoder( RI_WHEEL_RIGHT ) );
		leftEnc = FirFilter( leftWheelFilter, robot->getWheelEncoder( RI_WHEEL_LEFT ) );
		yTotal +=  WheelAverageY( rightEnc, leftEnc );
		
		if ( yTotal >= (.8 * ticks ) && speedCheck )
		{
			speed = RI_SLOWEST;
			speedCheck = false;
		}
		
		if ( yTotal >= ticks )
			break;
		/*
		predicted = UpdateKalman();
		
		printf("\n===KALMAN PREDICTION ARRAY===\n");
		for ( int i=0; i<9; i++ )
		{
			printf("%d: %f\n", i, predicted[i]);
		}
		
		#ifdef FILEDUMP
		#ifdef DUMP_NS
		filterDataFile << FirFilter( xFilter, CORRECT_NS_X(robot->X()) ) << " " << FirFilter( yFilter, CORRECT_NS_Y(robot->Y()) ) << "\n";
		#endif
		#endif
		//*/
		//printf("yTotal: %.3f\n", yTotal);
	}

	return OK;
}




// handle ctrl + c
void QuitHandler( int s )
{
	printf( "Caught Ctrl + C. Dying a (more) graceful death\n" );
	
	delete(robot);
	
	#ifdef FILEDUMP
	rawDataFile.close();
	filterDataFile.close();
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
	
	// initialize fir filters for each sensor
	if ( FAILED( InitializeFirFilters( robot ) ) )
	{
		std::cout << "Failed to initialize fir filters. Exiting" << std::endl;
		exit(-1);
	}
	
	
	// initialize kalman filter
	if ( FAILED( InitializeKalmanFilter( robot ) ) )
	{
		std::cout << "Failed to initialize kalman filter. Exiting" << std::endl;
		exit(-1);
	}
	
	
	#ifdef FILEDUMP
	rawDataFile.open("data/rawData.txt");
	filterDataFile.open("data/filterData.txt");
	#endif
	
	// set origin
	printf( "Gathering origin data...\n");
	SetOrigin();
	printf( "Origin set to (%.3f, %.3f) with heading %.3f\n", xNSOrigin, yNSOrigin, thetaCorrection );	
	
	TurnTo2(0.0,6);
	MoveToXY(0,150,5);
	MoveToXY(0,290,5);// base 1

	TurnTo(227.0);
	MoveToXY(123,186,5); // base 2
	
	TurnTo(0.0);
	MoveToXY(120,327,5); 
	TurnTo(275.0);
	MoveToXY(260,327,5);// base 3

	TurnTo(180);
	MoveToXY(265,20,5); // base 4
	
	TurnTo(110);
	MoveToXY(-30,-40,5); //base 0
	/*
	yEncCoord = 100;
	xEncCoord = 200;
	MoveTo2(0, 145,0);
	TurnTo(270);
	xEncCoord = 100;
	yEncCoord = 300;
	MoveTo2(0,145,0);

	//TurnTo(130);
	//measurement();
	
	

	/*
	printf("Navigating to 1st waypoint...\n");
	
	MoveTo(0, 145);
	xEncCoord = 0;
	yEncCoord = 145;
	
	printf("Navigating to 2nd waypoint...\n");
	TurnTo((float)230);
	
	MoveTo2( 63, 100,(float)230);
	xEncCoord = 65;
	yEncCoord = 100;
	
	printf("Navigating to 3rd waypoint...\n");
	TurnTo((float)323);
	
	MoveTo2( 123, 154,(float)323);
	xEncCoord = 123;
	yEncCoord = 154;
	
	printf( "Stopped at %.3f ticks (%.3f inches)\n", yTotal, (yTotal/TICKS_PER_INCH) );
	//*/
	
	// Clean up
	delete(robot);
	
	#ifdef FILEDUMP
	rawDataFile.close();
	filterDataFile.close();
	#endif
	
	return 0;
}