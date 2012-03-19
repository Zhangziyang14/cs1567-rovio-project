/** * * * * * * * * * * * * * * * * * * * * *
* Robot.cpp									*
* Created: 3/12/2012						*
* Authors: CJ McAllister, Yuxin Liu         *
*											*
* * * * * * * * * * * * * * * * * * * * * * */

#include "Robot.h"

using namespace std;

/******define CONSTANTS******/
#define ANGLE_WHEEL_RIGHT 0.523598776
#define ANGLE_WHEEL_LEFT 2.61799388
#define ANGLE_WHEEL_REAR 1.57079633
#define COS_R 0.8660254            // cos(30).
#define COS_L -0.8660254		   // cos(150).
#define SIN 0.5                    // sin(30) and sin(150).
#define PI 3.1415926
#define DIAMETER 29
#define ACTION_TURN 0x101/*action flag pass to update function*/
#define ACTION_MOVE 0x102/*action flag pass to update function*/
#define OK 0
#define FAIL -1

#define TURNTO_MARGIN 3
#define MOVETO_MARGIN 5

#define ROOM2_Theta 2.0
#define ROOM3_Theta 3.0
#define ROOM4_Theta 4.0
#define ROOM5_Theta 5.0

#define CM_PER_NS 0.013f
#define CM_PER_TICK 0.62
#define DEGREE_PER_TICK 2.1160
#define RADIAN_PER_TICK 0.0369


/******define utility macros******/
#define TODEGREE(x) ((float)((x)*57.29578)) //turn radian into degree
#define ABS(x) ((x)>0?(x):-(x))
#define TORADIAN(x) ((float)(x*0.0174533))//turn degree to radian
#define MINABS(x,y) (ABS(x)<ABS(y)?(x):(y))//return the one with lower absolute value


/****** Image Recognition Macros ******/
#define MY_PINK_LOW cvScalar(160, 0, 0)
#define MY_PINK_HIGH cvScalar(179, 255, 255)
#define SLOPE_RANGE 0.1

// Constructor
// Initializes values for Robot control
Robot::Robot(string name)
{
    robot = new RobotInterface(name,1);
    kf = new Kalman();
    xFilter = new Fir();
    yFilter = new Fir();
    rightFilter = new Fir();
    leftFilter = new Fir();
    rearFilter = new Fir();
}

// Destructor
// Frees memory of Robot object
Robot::~Robot()
{
    delete(robot);
    delete(kf);
    delete(xFilter);
    delete(yFilter);
    delete(rightFilter);
    delete(leftFilter);
    delete(rearFilter);

}

void Robot::NS_Rotate(int room){ 
    switch(room)
	{
		case 2: 
		{

			break;
		}
		case 3: 
		{

			break;
		}
		case 4: 
		{
            break;
		}
		case 5: 
		{

			break;
		}
	}


}
void Robot::NS_Scale(){ 
    
    currNSX = currNSX * CM_PER_NS;
    currNSY = currNSY * CM_PER_NS;

}
void Robot::NS_Align(int room){ 
    switch(room)
	{
		case 2: 
		{
            
			break;
		}
		case 3: 
		{
            
			break;
		}
		case 4: 
		{
            break;
		}
		case 5: 
		{
            
			break;
		}
	}

}
void Robot::updateNS(int flag){
    if(ACTION_MOVE == flag){
        float deltaTheta;
        int acceptableRange = TORADIAN(3);//acceptable range for deltaTheta
    
        while(robot->update() != RI_RESP_SUCCESS) {cout << "Failed to update sensor information!" << endl;}
    
    
    
        if (roomID!=robot->RoomID()) {
            xFilter->reset();
            yFilter->reset();
            roomID = robot->RoomID();
        }
    
        currNSX = xFilter->getValue(robot->X());
        currNSY = yFilter->getValue(robot->Y());
    
        deltaTheta = robot->Theta() - finalTheta;
        if (ABS(deltaTheta) <= acceptableRange) {
            currNSTheta = finalTheta;
        }else{
            currNSTheta = finalTheta + deltaTheta;
        }
    
        this->NS_Rotate(roomID);
        this->NS_Scale();
        this->NS_Align(roomID);
        
    }else if(ACTION_TURN == flag){
        int step = 0;
		float curD = 0;
		float curT = 0;
        float sampleTheta;
        float newNSd;

        
        do{
            sampleTheta = robot->Theta();//get current theta
			newNSd = TODEGREE(sampleTheta);//turn it into degrees
            
            curT +=sampleTheta;
            curD += newNSd;
            
        }while (step<10);
        
        currNSTheta = CorrectTheta(curT/10,roomID);//take average of last 10 readings
        
    }
    
    
    
    
}

void Robot::updateWE(int flag){ 
    if(ACTION_MOVE == flag) {//it's a move update
		float theta = finalTheta;
		int r = robot -> getWheelEncoder(RI_WHEEL_RIGHT);
		int l = robot -> getWheelEncoder(RI_WHEEL_LEFT);
		int b = robot -> getWheelEncoder(RI_WHEEL_REAR);
        
		float filterR =  rightFilter->getValue(r);
		float filterL =  leftFilter->getValue(l);
		float filterB =  rearFilter->getValue(b);
        
        //		printf("r %d, l %d, b %d,fr %5.2f, fl %5.2f, fb %5.2f\n",r,l,b,filterR,filterL,filterB);
        
		float moveY = WheelAverageY(filterR, filterL) * CM_PER_TICK;//get x contribution after filter  
		float moveX = WheelAverageX(filterR, filterL) * CM_PER_TICK;//get y contribution after filter
        
        //		printf("moveX %5.2f ",moveX);
        
		currWX += -cos(theta)*moveY-cos(theta)*moveX;
		currWY += -sin(theta)*moveX-sin(theta)*moveY;
		
        //		printf(" Coord: %5.2f,%5.2f,%5.2f,%5.2f,%4.3f\t",preWX,preWY,curWX,curWY,curWTheta);
	}
	else if(ACTION_TURN == flag) {//it's a turn update
		int b = robot -> getWheelEncoder(RI_WHEEL_REAR);
		float filterB = rearFilter->getValue(b);
		
		float deltaTheta = filterB * RADIAN_PER_TICK;	//calcualte estimate 
		currWTheta += deltaTheta;
	}


}

void Robot::updateKalman(){
    float currNS[3];
    float currWE[3];
    float kalmanResult[3];
    
    currNS[0] = currNSX;
    currNS[1] = currNSY;
    currNS[2] = currNSTheta;
    
    currWE[0] = currWX;
    currWE[1] = currWY;
    currWE[2] = currWTheta;
    
    kf->rovioKalmanFilter(currNS, currWE, kalmanResult);
    
    finalX = kalmanResult[0];
    finalY = kalmanResult[1];
    finalTheta = kalmanResult[2];

}

void Robot::Init(){ 
    
    /*********************
     Initialize Fir Filter
     *********************/
    for ( int i=0; i<3; i++ )
	{
        while(robot->update() != RI_RESP_SUCCESS) {cout << "Failed to update sensor information!" << endl;}
		xFilter->getValue(robot->X());
		yFilter->getValue(robot->Y());
	}
    
    /************************
     Initialize Kalman Filter
     ************************/
    float initPose[3];
	float velocity[3];
	float predicted[9];
	
	float firX, firY;
	
	int deltat = .1;
	float thetaSum = 0.0;
    
	if ( initPose == NULL || velocity == NULL || predicted == NULL || kf == NULL )
	{
		printf("Allocation failure, exiting\n");
		exit(-1);
	}
	
    while(robot->update() != RI_RESP_SUCCESS) {cout << "Failed to update sensor information!" << endl;}
	
	// acquire fir-filtered x/y vals
	firX = xFilter->getValue(robot->X());
	firY = yFilter->getValue(robot->Y());
	
	// correct x/y
	initPose[0] = 0;
	initPose[1] = 0;
	
	for ( int i=0; i<10; i++ )
	{
		while(robot->update() != RI_RESP_SUCCESS) {cout << "Failed to update sensor information!" << endl;}
		thetaSum += CorrectTheta( robot->Theta(), robot->RoomID() );
	}
	
	initPose[2] = thetaSum / 10;
	
	velocity[0] = 0.0;
	velocity[1] = 0.0;
	velocity[2] = 0.0;
	
    #ifdef DEBUG
    printf( "Initial Pose:\nX: %.3f Y: %.3f Theta(Rad): %.3f Theta(Deg): %.3f\n", initPose[0], initPose[1], initPose[2], TODEGREE(initPose[2]) );
    #endif
	
	kf->initialize(initPose, velocity, deltat);
	

	
    
}

void Robot::InitCamera()
{
	// Setup the camera
	if(robot->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, RI_CAMERA_DEFAULT_CONTRAST, 5, RI_CAMERA_RES_640, RI_CAMERA_QUALITY_HIGH)) {
		cout << "Failed to configure the camera!" << std::endl;
		exit(-1);
	}
	
	// Create a window to display the output
	cvNamedWindow("Pink Squares", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Threshold", CV_WINDOW_AUTOSIZE);
	
	// Create an images
	m_pImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	m_pHsv = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	m_pThreshold = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	cvZero(m_pImage);
	cvZero(m_pHsv);
	cvZero(m_pThreshold);

	// Move the head up to the middle position
	robot->Move(RI_HEAD_MIDDLE, RI_FASTEST);
}

void Robot::CamNav()
{
	int direction = RI_MOVE_FORWARD;
	bool adjust_needed = false;
	
	double slope = 0.0;
	CvPoint centerPoint;
	
	squares_t *biggest = NULL;

	do
	{
		if( !adjust_needed )
			robot->Move(RI_MOVE_FORWARD, RI_FASTEST);

		if(robot->update() != RI_RESP_SUCCESS) {
			std::cout << "Failed to update sensor information!" << std::endl;
			continue;
		}

		// Get the current camera m_pImage and display it
		if(robot->getImage(m_pImage) != RI_RESP_SUCCESS) {
			std::cout << "Unable to capture an image!" << std::endl;
			continue;
		}

		// construct filtered m_pImage
		squares_t *pinkSquares = FindSquares( RC_PINK );
		//squares_t *yelSquares = FindSquares( RC_YELLOW );
		
		biggest = GetBiggestPair( pinkSquares );

		// if no pairs found
		if ( biggest == NULL )
			biggest = GetBiggestSquares( pinkSquares );
		
		// if 2 squares found, draw line connecting them
		if( biggest != NULL && biggest->next != NULL )
		{
			DrawSquareLine( biggest, &slope, &centerPoint );
			DrawOnSquares( biggest, CV_RGB(255, 0, 0) );
		}

		// draw line down image center
		cvLine(m_pImage, cvPoint(320, 480), cvPoint(320, 0), CV_RGB(255, 0, 255), 3);

		// Display the drawn-on image and the threshold
		cvShowImage("Pink Squares", m_pImage);
		cvShowImage("Threshold", m_pThreshold);
		
		// Update the UI
		cvWaitKey(5);

		printf("slope: %.3f\n", slope);
		// adjustment required if slope is outside range
		if ( slope >= SLOPE_RANGE || slope <= -SLOPE_RANGE )
		{
			// left turn required
			if ( slope > 0 )
				direction = RI_TURN_LEFT;
			// right turn required
			else
				direction = RI_TURN_RIGHT;
			
			robot->Move(direction, 4);
			robot->Move(RI_STOP, 1);

			adjust_needed = true;
		}
		else
		{
			adjust_needed = false;
		}

		// Release the square and image data
		squares_t *sq_tmp;
		while(pinkSquares != NULL) {
			sq_tmp = pinkSquares->next;
			delete(pinkSquares);
			pinkSquares = sq_tmp;
		}

		while( biggest != NULL)
		{
			sq_tmp = biggest->next;
			delete(biggest);
			biggest = sq_tmp;
		}

		/*
		printf("X: %.3f Y: %.3f Theta: %.3f Room %d\n", robot->X(), robot->Y(), robot->Theta(), robot->RoomID());
		printf("Right: %d Left: %d Rear: %d\n", robot->getWheelEncoderTotals(RI_WHEEL_RIGHT), robot->getWheelEncoderTotals(RI_WHEEL_LEFT), robot->getWheelEncoderTotals(RI_WHEEL_REAR)); 
		*/
	}while( 1 );
}

squares_t *Robot::FindSquares( int color )
{
	CvScalar lineColor;
	squares_t *squares;
	
	IplImage *pink1 = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
	IplImage *pink2 = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
	cvZero(pink1);
	cvZero(pink2);

	// Convert the m_pImage from RGB to HSV
    cvCvtColor(m_pImage, m_pHsv, CV_BGR2HSV);

	// Pick out only the pink or yellow color from the m_pImage
	if ( color == RC_PINK )
	{
		// filter threshold image
		cvSmooth(m_pHsv, m_pHsv, CV_MEDIAN, 7, 7);

		cvInRangeS(m_pHsv, cvScalar(150, 100, 100), cvScalar(180, 255, 255), pink1);
		cvInRangeS(m_pHsv, cvScalar(0, 100, 100), cvScalar(15, 255, 255), pink2);
		
		cvOr(pink1, pink2, m_pThreshold);

		cvReleaseImage(&pink1);
		cvReleaseImage(&pink2);

		lineColor = CV_RGB(0, 255, 0);
	}
	else if ( color == RC_YELLOW )
	{
		cvInRangeS(m_pHsv, RC_YELLOW_LOW, RC_YELLOW_HIGH, m_pThreshold);
		lineColor = CV_RGB(0, 0, 255);
	}
	else
	{
		printf("Bad color code. Exiting.\n");
		exit(-1);
	}
	
	// Find the squares in the m_pImage
	squares = robot->findSquares(m_pThreshold, 300);

	DrawOnSquares(squares, lineColor);

	return squares;
}

squares_t *Robot::GetBiggestPair( squares_t *squares)
{
	squares_t *sq_tmp = squares;
	vector<squares_t *> sq_vector;
	vector<squares_t *> sq_pairs_vector;
	squares_t *pair1 = (squares_t *)malloc(sizeof(squares_t));
	squares_t *pair2 = (squares_t *)malloc(sizeof(squares_t));

	// if 1 or no squares, return NULL
	if ( squares == NULL || squares->next == NULL )
	{
		return NULL;
	}

	// put squares into vector
	while( sq_tmp != NULL )
	{
		sq_vector.push_back(sq_tmp);
		sq_tmp = sq_tmp->next;
	}

	// iterate through vector, pulling out pairs
	for( int i=0; i<sq_vector.size(); i++ )
	{
		for( int j=i+1; j<sq_vector.size(); j++ )
		{
			// determine pairs by dist between y coords, eliminate doubles by x coord
			int yDist = abs(sq_vector[i]->center.y - sq_vector[j]->center.y);
			int xDist = abs(sq_vector[i]->center.x - sq_vector[j]->center.x);

			if ( yDist < 20 && xDist > 20 )
			{
				// copy pair values
				pair2->area = sq_vector[j]->area;
				pair2->center.x = sq_vector[j]->center.x;
				pair2->center.y = sq_vector[j]->center.y;
				pair2->next = NULL;
				pair1->area = sq_vector[i]->area;
				pair1->center.x = sq_vector[i]->center.x;
				pair1->center.y = sq_vector[i]->center.y;
				pair1->next = pair2;

				sq_pairs_vector.push_back( pair1 );
			}
		}
	}
	
	// if no pairs found, return NULL
	if ( sq_pairs_vector.empty() )
		return NULL;

	// determine largest pair
	int prev_largest_area = 0;
	int largest_index = 0;
	for( int i=0; i<sq_pairs_vector.size(); i++ )
	{
		if ( sq_pairs_vector[i]->area > prev_largest_area )
		{
			prev_largest_area = sq_pairs_vector[i]->area;
			largest_index = i;
		}
	}

#ifdef DEBUG
		int i=0;
		squares_t *sq_temp_debug = sq_pairs_vector[largest_index];

		//print biggest info
		printf("\n===== BIGGEST PAIR INFO =====\n");

		while( sq_temp_debug != NULL )
		{
			printf("Square %d:\n", i);
			printf("Center: (%d, %d)\n", sq_temp_debug->center.x, sq_temp_debug->center.y);
			printf("Area: %d\n", sq_temp_debug->area);
		
			sq_temp_debug = sq_temp_debug->next;
			i++;
		}
#endif

	return sq_pairs_vector[largest_index];
}

squares_t *Robot::GetBiggestSquares( squares_t *squares )
{
	squares_t *sq_tmp = squares;
	squares_t *biggest = (squares_t *)malloc(sizeof(squares_t));
	squares_t *second = (squares_t *)malloc(sizeof(squares_t));

	biggest->area = 0;
	second->area = 0;

	// check for no squares
	if ( squares == NULL )
	{
		return NULL;
	}

	// find biggest square
	while( sq_tmp != NULL )
	{
		// copy contents of sq_tmp if bigger than current biggest
		if( sq_tmp->area > biggest->area )
		{
			biggest->area = sq_tmp->area;
			biggest->center.x = sq_tmp->center.x;
			biggest->center.y = sq_tmp->center.y;
			biggest->next = NULL;
		}

		sq_tmp = sq_tmp->next;
	}
	sq_tmp = squares;
	
	// if only 1 square, return biggest w/ count 1
	if ( squares->next == NULL )
		return biggest;

	// find second biggest square
	while ( sq_tmp != NULL )
	{
		int dist = abs(biggest->center.x - sq_tmp->center.x);

		// find second largest square, ignore squares that are too close
		if( sq_tmp->area > second->area && sq_tmp->area < biggest->area && dist > 10 )
		{
			second->area = sq_tmp->area;
			second->center.x = sq_tmp->center.x;
			second->center.y = sq_tmp->center.y;
			second->next = NULL;

			biggest->next = second;
		}

		sq_tmp = sq_tmp->next;
	}

#ifdef DEBUG
		int i=0;
		squares_t *sq_temp_debug = biggest;

		//print biggest info
		printf("\n===== BIGGEST SQUARES INFO =====\n");

		while( sq_temp_debug != NULL )
		{
			printf("Square %d:\n", i);
			printf("Center: (%d, %d)\n", sq_temp_debug->center.x, sq_temp_debug->center.y);
			printf("Area: %d\n", sq_temp_debug->area);
		
			sq_temp_debug = sq_temp_debug->next;
			i++;
		}
#endif

	return biggest;
}

/**
 * Draws a line between the 2 biggest squares
 * 
 * Returns slope and centerPoint output params
 */
void Robot::DrawSquareLine( squares_t *biggest, double *slope, CvPoint *centerPoint )
{
	//draw horizontal line connecting squares
	CvPoint pt1, pt2;
	
	pt1.x = biggest->center.x;
	pt1.y = biggest->center.y;
	pt2.x = biggest->next->center.x;
	pt2.y = biggest->next->center.y;

	cvLine(m_pImage, pt1, pt2, CV_RGB(0, 0, 255), 3);

	*slope = -((double)pt2.y - (double)pt1.y) / ((double)pt2.x - (double)pt1.x);

	//draw vertal line  between squares
	int xCenter = (biggest->center.x + biggest->next->center.x)/2;
	int yCenter = (biggest->center.y + biggest->next->center.y)/2;

	pt1.x = xCenter;
	pt1.y = yCenter + 10;
	pt2.x = xCenter;
	pt2.y = yCenter - 10;
		
	cvLine(m_pImage, pt1, pt2, CV_RGB(0, 0, 127), 3);

	*centerPoint = cvPoint(xCenter, yCenter);
}

void Robot::DrawOnSquares( squares_t *squares, CvScalar lineColor )
{
	CvPoint pt1, pt2;
	int sq_amt;
	squares_t *sq_temp = squares;

	// draw until end of list reached
	while( sq_temp != NULL ) {
		// Draw an X marker on the m_pImage
		sq_amt = (int) (sqrt(sq_temp->area) / 2);	

		// Upper Left to Lower Right
		pt1.x = sq_temp->center.x - sq_amt;
		pt1.y = sq_temp->center.y - sq_amt;
		pt2.x = sq_temp->center.x + sq_amt;
		pt2.y = sq_temp->center.y + sq_amt;
		cvLine(m_pImage, pt1, pt2, lineColor, 3, CV_AA, 0);

		// Lower Left to Upper Right
		pt1.x = sq_temp->center.x - sq_amt;
		pt1.y = sq_temp->center.y + sq_amt;
		pt2.x = sq_temp->center.x + sq_amt;
		pt2.y = sq_temp->center.y - sq_amt;
		cvLine(m_pImage, pt1, pt2, lineColor, 3, CV_AA, 0);

		sq_temp = sq_temp->next;
	}
}

int Robot::ListLength( squares_t *list )
{
	squares_t *tmp = list;
	int i;

	while ( tmp != NULL )
	{
		i++;
		tmp = tmp->next;
	}

	return i;
}

void Robot::ReadData(){ 
    long sumd = 0;
	long sumX = 0;
	long sumY = 0;
	int step = 1;
    
	do {
		// Update the robot's sensor information.
		while(robot->update() != RI_RESP_SUCCESS) {cout << "Failed to update sensor information!" << endl;}
		
		int degree = TODEGREE(CorrectTheta(robot->Theta(),roomID));
		int x = xFilter->getValue(robot->X());
		int y = yFilter->getValue(robot->Y());
		roomID = robot->RoomID();
		
		sumd += degree;
		sumX += x;
		sumY += y;
        		printf("X: %5d, Y: %5d, degree: %4d, avg: %6d|%6d|%4d, signal %d-%6d\n",
        			   x,y,degree, sumX/step,sumY/step,sumd/step, roomID,robot->NavStrengthRaw());
        
		step++;
	} while(1);
    
}

void Robot::MoveTo(float targetX, float targetY){
    PID *pid = new PID();   
	float XYRange = sqrt(MOVETO_MARGIN); 
	float Xoffset=0;
	float Yoffset=0;
	float thetaOffset = TORADIAN(5);
	bool NewTarget = false;    
	float disTravel;
	float disoffset;
    float currX = finalX;
    float currY = finalY;;
    float currTheta = finalTheta;
    float vel[3]; // velocity array
    float pid_val;
    
	//total distance we need to travel
	float totalDistance = sqrt(pow(targetX-currX,2) + pow(targetY-currY,2));
	float beginX = currX, beginY = currY;
	int move_flag = RI_MOVE_FORWARD;
	int move_speed = 1;//initial speed set to 1
    
	do{ 
        /*********************************
          update X, Y Theta
         *********************************/
        updateNS(ACTION_MOVE);
        updateWE(ACTION_MOVE);
        
        vel[0] = move_speed * -sin(currTheta) * CM_PER_TICK;
        vel[1] = move_speed * cos(currTheta) * CM_PER_TICK;
        kf->rovioKalmanFilterSetVelocity(vel);
        updateKalman();
        
        currX =finalX;
        currY = finalY;
        currTheta = finalTheta;
        
        
        
		Xoffset = targetX - currX;
		Yoffset = targetY - currY;
        
        //distance we travel so far
		disTravel = sqrt(pow(beginX-currX,2)+ pow(beginY-currY,2));
		//how far we still need to travel
        disoffset = (sqrt(pow(Xoffset,2)+pow(Yoffset,2)));
		//if travel distance > total distance then disoffset becomes negative
		if(disTravel>totalDistance){
			disoffset = (-1)*disoffset;
        }
        //PID control
        pid_val = pid->control(disoffset);
        
		printf(" offset: %6.2f, \n",disoffset);
        
		//if distance offset is less than margin, done
		if(disoffset <= XYRange) {
			if(!robot->IR_Detected()){//issue stop command
				robot->Move(RI_STOP,move_speed);
			}
			
			NewTarget = true;//we dont need to check new target
			break;
		}
        
		//If we already reach targetX or targetY
		if((ABS(Xoffset)<=XYRange || ABS(Yoffset)<=XYRange) && NewTarget==false) {			
            NewTarget = true;            //we already check it before
            
			if(ABS(Xoffset)<=XYRange) {    //we get to targeX, but not target Y
				if(Yoffset>0) {                    //current Y is too large, then we should face 270 degree
					if(ABS(currTheta-TORADIAN(0))>thetaOffset){//if we are not facing 270 degree
						printf("----target %5.2f, %5.2f--------Im at %5.2f,%5.2f, need to face 270\n",
							   targetX,targetY,currX,currY);
						TurnTo(0.0);
						MoveTo(targetX,targetY);//move to a new destination
						break;
					}
				}
				else{
					if(ABS(currTheta-TORADIAN(180))>thetaOffset){//if we are not facing 270 degree
						printf("----target %5.2f, %5.2f--------Im at %5.2f,%5.2f, need to face 270\n",
							   targetX,targetY,currX,currY);
						TurnTo(180.0);
						MoveTo(targetX,targetY);//move to a new destination
						break;
					}
                    
				}
			}
			else if((ABS(Yoffset)<=XYRange)){
				if(Xoffset>0) {  
					if(ABS(currTheta-TORADIAN(270))>thetaOffset){//if we are not facing 270 degree
						printf("----target %5.2f, %5.2f--------Im at %5.2f,%5.2f, need to face 270\n",
							   targetX,targetY,currX,currY);
						TurnTo((float)270.0);
						MoveTo(targetX,targetY);//move to a new destination
						break;
					}
                    
				}
				else{
					if(ABS(currTheta-TORADIAN(90))>thetaOffset){//if we are not facing 270 degree
						printf("----target %5.2f, %5.2f--------Im at %5.2f,%5.2f, need to face 270\n",
							   targetX,targetY,currX,currY);
						TurnTo((float)90.0);
						MoveTo(targetX,targetY);//move to a new destination
						break;
					}
				}
                
			}
            
            
		}
        
        if(pid_val<=50) {	//if pid value is less then 50
			move_speed = 8;
		}
		else if(pid_val<100) {//if pid value is less then 100 and greater than 50
			move_speed = 5;
		}
		else {//else set speed to 8
			move_speed = 1;
		}
        
        
		if(disoffset<0) {
			move_flag = RI_MOVE_BACKWARD;
		}
		else {
			move_flag = RI_MOVE_FORWARD;
		}
        
        //do the move
		if(!robot->IR_Detected()) { 
			robot -> Move(move_flag, move_speed);
		}

		// check for path center
		CamNav();
        
	}while(1);

    

}

void Robot::TurnTo(float target){ 
    
    float degreeCurrent;
	float offset = TURNTO_MARGIN; 
	float diff;//diff between current and target
	PID *pid = new PID();
	bool multiple_sample = true;//indicate if we need to take multiple sample
	int turn_speed = 8;
	int turn_flag = RI_TURN_LEFT;//set initial turn flag to turn left
    float pid_val; //pid value
	float vel[3]; // velocity array
	
	do {
        // data sample collecting
        if(multiple_sample){
        
            updateNS(ACTION_TURN);
            updateWE(ACTION_TURN);
            
            vel[2] = turn_speed*RADIAN_PER_TICK;
            kf->rovioKalmanFilterSetVelocity(vel);
            updateKalman();
            degreeCurrent = TODEGREE(finalTheta);
            
            diff = MINABS(target-degreeCurrent, 360-degreeCurrent+target);//get diff between target and current
            printf("-1target:%4.2f curr:%4.2f diff:%4.2F\n",target,degreeCurrent,diff);
        }
        //PID control
        pid_val = pid->control(diff);
        
        //break loop condition if diff is within a preset acceptable offset
		if(ABS(diff)<=offset)
			break;
        
		//if absolute value of diff >= 20
		if(ABS(diff) >= 20) {
			turn_flag = diff>0? RI_TURN_RIGHT_20DEG: RI_TURN_LEFT_20DEG;//set turn flag
			turn_speed = ABS(pid_val)>20?4:2;//set turn speed
            
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
            while(robot->update() != RI_RESP_SUCCESS) {cout << "Failed to update sensor information!" << endl;}

			robot -> Move(turn_flag, turn_speed);
            
            while(robot->update() != RI_RESP_SUCCESS) {cout << "Failed to update sensor information!" << endl;}
			
			if(!robot->IR_Detected()) {
				robot -> Move(RI_STOP, turn_speed);
			}
            
		}		
		
		printf("target:%4.2f curr:%4.2f diff:%4.2F\n\n",target,degreeCurrent,diff);
	}
	while(1);

}

float Robot::CorrectTheta(float oldTheta, int roomID){
    float newTheta;
	
	switch(roomID) {
		case 2: //room 2
			newTheta = 360+255 - TODEGREE(oldTheta);
			break;
		case 3: //room3
			newTheta = 360+180 - TODEGREE(oldTheta);
			break;
		case 4: //room 4
			newTheta = 360+275 - TODEGREE(oldTheta);
			break;
		case 5: //room 5
			newTheta = 360+4 - TODEGREE(oldTheta);
			break;
		default:
			//printf("room %d, Man, are you sure you are not in outer space!\n",roomID);
			return 0;
	}
	
	return TORADIAN(fmod( newTheta, (float)360 ));

}

float Robot::WheelAverageX( float rightEncoder, float leftEncoder )
{
	float rightFinal, leftFinal;
	rightFinal = rightEncoder * cos( ANGLE_WHEEL_RIGHT );
	leftFinal = leftEncoder * cos( ANGLE_WHEEL_LEFT );
	//printf( "WheelAverageX():\nrightFinal: %.3f leftFinal: %.3f\n", rightFinal, leftFinal );
	return (rightFinal + leftFinal) / 2;
}

// returns average of computed left and right encoder y-axis motion
float Robot::WheelAverageY( float rightEncoder, float leftEncoder )
{
	float rightFinal, leftFinal;
	rightFinal = rightEncoder * sin( ANGLE_WHEEL_RIGHT );
	leftFinal = leftEncoder * sin( ANGLE_WHEEL_LEFT );
	return (rightFinal + leftFinal) / 2;
}