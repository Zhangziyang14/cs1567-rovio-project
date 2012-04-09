/** * * * * * * * * * * * * * * * * * * * * *
* Camera.cpp								*
* Created: 3/23/2012						*
* Authors: CJ McAllister, Yuxin Liu         *
*											*
* * * * * * * * * * * * * * * * * * * * * * */

#include <iostream>
#include <fstream>

#include "Camera.h"

#define SLOPE_TOLERANCE	 0.1
#define OFFSET_TOLERANCE 30

#define FSM_NO_SQUARES	0x00
#define FSM_ONE_SQUARE  0x01
#define FSM_PAIR		0x02
#define FSM_NO_PAIR		0x03

using namespace std;


Camera::Camera()
{	
    m_pImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
    m_pHsv = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
    m_pThreshold = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);

    m_CvPCenterPoint = cvPoint(0, 0);
}

Camera::~Camera()
{
	cvReleaseImage(&m_pImage);
	cvReleaseImage(&m_pHsv);
	cvReleaseImage(&m_pThreshold);
}

void Camera::InitCamera( RobotInterface *robot )
{
	//set RobotInterface pointer to member variable
	m_robot = robot;
		
    // Setup the camera
    if(m_robot->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, RI_CAMERA_DEFAULT_CONTRAST, 5, RI_CAMERA_RES_640, RI_CAMERA_QUALITY_HIGH)) {
        cout << "Failed to configure the camera!" << std::endl;
        exit(-1);
    }
        
    // Create a window to display the output
    cvNamedWindow("Pink Squares", CV_WINDOW_AUTOSIZE);
#ifdef DEBUG
    cvNamedWindow("Threshold", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("pink1", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("pink2", CV_WINDOW_AUTOSIZE);
#endif
        
    // Initialize correction data
    m_iDirection = RI_MOVE_FORWARD;
    m_bAdjust = false;
    m_dSlope = 0.0;

    // zero out images
    cvZero(m_pImage);
    cvZero(m_pHsv);
    cvZero(m_pThreshold);

    // Move the head up to the middle position
    m_robot->Move(RI_HEAD_MIDDLE, RI_FASTEST);
}

/** * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * void CamCenter()													*
  *																*
  * Centers robot in path using squares found in camera input	*
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Camera::CamCenter()
{
	int fsmCode;
	m_bAdjust = true;

	while( m_bAdjust ){

        if(m_robot->update() != RI_RESP_SUCCESS) {
            std::cout << "Failed to update sensor information!" << std::endl;
            continue;
        }
		
        // Get the current camera m_pImage and display it
        if(m_robot->getImage(m_pImage) != RI_RESP_SUCCESS) {
            std::cout << "Unable to capture an image!" << std::endl;
            continue;
        }
        
		// handle biggest squares based on fsmCode
        m_vSquares = GetSortedSquares( &fsmCode );

		switch( fsmCode )
		{
			case FSM_NO_SQUARES:
				continue;
			case FSM_ONE_SQUARE:
				m_vBiggest.push_back(m_vSquares[0]);
				break;
			case FSM_PAIR:
				m_vBiggest.push_back(m_vSquares[0]);
				m_vBiggest.push_back(m_vSquares[1]);
				break;
			case FSM_NO_PAIR:
				m_vBiggest.push_back(m_vSquares[0]);
				m_vBiggest.push_back(m_vSquares[1]);
				break;
			default:
				cout << "FSM error. Exiting" << endl;
				exit(-1);
		}

        // Display the image
        cvLine(m_pImage, cvPoint(320, 480), cvPoint(320, 0), CV_RGB(255, 0, 255), 3);
        cvShowImage("Pink Squares", m_pImage);

#ifdef DEBUG
        cvShowImage("Threshold", m_pThreshold);
#endif
                
        // Update the UI
        cvWaitKey(5);

#ifdef DEBUG
        printf("m_dSlope: %.3f\n", m_dSlope);
        printf("centerPoint.x: %d\n", m_CvPCenterPoint.x);
#endif   
		// determine if adjust needed based on squares in m_vBiggest
        DetermineAdjustment( );
		
        if( m_bAdjust )
        {
            m_robot->Move(m_iDirection, 3);
            m_robot->Move(RI_STOP, 1);
        }
	}
}

/** * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * vector<squares_t *> GetSortedSquares( int *fsmCode )						*
  *																			*
  * Locates and sorts pink squares in camera input, draws green X's on all.	*
  * Determines how many found and if a pair is present.						*
  * Returns code for state machine, also output parameter is squares found	*
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
vector<squares_t *> Camera::GetSortedSquares( int *fsmCode )
{
	squares_t *tmpSquares;
	vector<squares_t *> vSquares;
        
    IplImage *pink1 = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
    IplImage *pink2 = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
    cvZero(pink1);
    cvZero(pink2);

    // Convert the m_pImage from RGB to HSV
    cvCvtColor(m_pImage, m_pHsv, CV_BGR2HSV);

    // filter and combine for threshold image
    cvSmooth(m_pHsv, m_pHsv, CV_MEDIAN, 7, 7);
    cvInRangeS(m_pHsv, cvScalar(150, 100, 100), cvScalar(180, 255, 255), pink1);
    cvInRangeS(m_pHsv, cvScalar(0, 100, 100), cvScalar(15, 255, 255), pink2);
    cvOr(pink1, pink2, m_pThreshold);

#ifdef DEBUG
        cvShowImage("pink1", pink1);
        cvShowImage("pink2", pink2);
#endif

    cvReleaseImage(&pink1);
    cvReleaseImage(&pink2);
        
    // Find and sort the squares in the m_pImage, then draw X's
    tmpSquares = m_robot->findSquares(m_pThreshold, 300);
	MergeSortSquares( &tmpSquares );
	
	*fsmCode = DetermineFSMState(tmpSquares);

	// put squares into vector
	while ( tmpSquares != NULL && tmpSquares->next != NULL )
	{
		vSquares.push_back(tmpSquares);
		tmpSquares = tmpSquares->next;
	}

    DrawXOnSquares(vSquares, CV_RGB(0, 255, 0));

	return vSquares;
}

/** * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * void MergeSortSquares( squares_t *unsorted_squares )			*
  *																	*
  * Takes in an unsorted linked list of squares_t * and returns as	*
  * a sorted vector<squares_t *>									*
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Camera::MergeSortSquares( squares_t **unsorted_squares )
{
	squares_t *head = *unsorted_squares;
	squares_t *a;
	squares_t *b;
 
	// Base case
	if ((head == NULL) || (head->next == NULL))
		return;
 
	// Split head into 'a' and 'b' sublists
	SplitSquares(head, &a, &b); 
 
	// Recursively sort sublists
	MergeSortSquares(&a);
	MergeSortSquares(&b);
 
	// merge sorted lists together
	*unsorted_squares = MergeSquares(a, b);
}

void Camera::SplitSquares(squares_t *source, squares_t **frontRef, squares_t **backRef)
{
	squares_t *big;
	squares_t *small;

	// length < 2 cases
	if (source==NULL || source->next==NULL)
	{
		*frontRef = source;
		*backRef = NULL;
	}
	else
	{
		small = source;
		big = source->next;
 
		// Advance 'big' two nodes, and advance 'small' one node
		while (big != NULL)
		{
			big = big->next;
			if (big != NULL)
			{
			small = small->next;
			big = big->next;
			}
		}
 
		// 'small' is before midpoint, so split it in two at that point
		*frontRef = source;
		*backRef = small->next;
		small->next = NULL;
	}
}

squares_t *Camera::MergeSquares( squares_t *a, squares_t *b )
{
	squares_t *result = NULL;
 
	/* Base cases */
	if (a == NULL)
		return b;
	else if (b==NULL)
		return a;
 
	/* Pick either a or b, and recurse */
	if (a->area >= b->area)
	{
		result = a;
		result->next = MergeSquares(a->next, b);
	}
	else
	{
		result = b;
		result->next = MergeSquares(a, b->next);
	}
	return result;
}

int Camera::DetermineFSMState( squares_t *squares )
{
	int i=0;

#ifdef DEBUG
	cout << "DetermineFSMState:" << endl;
	squares_t *tmp = squares;
	while(tmp != NULL && tmp->next != NULL){
		cout << "square" << i << " area: " << tmp->area << " coords: (" << tmp->center.x << ", " << tmp->center.y << ")" << endl;
		tmp = tmp->next;
		i++;
	}
#endif

	// no squares found
	if ( squares == NULL )
		return FSM_NO_SQUARES;

	// 1 square found
	if (squares->next == NULL )
		return FSM_ONE_SQUARE;

	// pair found
	if ( abs(squares->center.y - squares->next->center.y) <= 15 )
		return FSM_PAIR;

	// 2+ squares, but biggest not paired
	else
		return FSM_NO_PAIR;
}

/**
 * Draws a line between the 2 biggest squares
 * 
 * Returns slope and centerPoint output params
 */
void Camera::DrawSquareLine( vector<squares_t *> biggest, double *slope, CvPoint *centerPoint )
{
        //draw horizontal line connecting squares
        CvPoint pt1, pt2;
        
        pt1.x = biggest[0]->center.x;
        pt1.y = biggest[0]->center.y;
        pt2.x = biggest[1]->next->center.x;
        pt2.y = biggest[1]	->next->center.y;

        cvLine(m_pImage, pt1, pt2, CV_RGB(0, 0, 255), 3);

        *slope = -((double)pt2.y - (double)pt1.y) / ((double)pt2.x - (double)pt1.x);

        //draw vertal line  between squares
        int xCenter = (biggest[0]->center.x + biggest[1]->center.x)/2;
        int yCenter = (biggest[0]->center.y + biggest[1]->center.y)/2;

        pt1.x = xCenter;
        pt1.y = yCenter + 10;
        pt2.x = xCenter;
        pt2.y = yCenter - 10;
                
        cvLine(m_pImage, pt1, pt2, CV_RGB(0, 0, 127), 3);

        *centerPoint = cvPoint(xCenter, yCenter);
}

void Camera::DrawXOnSquares( vector<squares_t *> squares, CvScalar lineColor )
{
        CvPoint pt1, pt2;
        int sq_amt;

        // draw until end of list reached
        for( int i=0; i<squares.size(); i++ ) {
                // Draw an X marker on the m_pImage
                sq_amt = (int) (sqrt(squares[i]->area) / 2);       

                // Upper Left to Lower Right
                pt1.x = squares[i]->center.x - sq_amt;
                pt1.y = squares[i]->center.y - sq_amt;
                pt2.x = squares[i]->center.x + sq_amt;
                pt2.y = squares[i]->center.y + sq_amt;
                cvLine(m_pImage, pt1, pt2, lineColor, 3, CV_AA, 0);

                // Lower Left to Upper Right
                pt1.x = squares[i]->center.x - sq_amt;
                pt1.y = squares[i]->center.y + sq_amt;
                pt2.x = squares[i]->center.x + sq_amt;
                pt2.y = squares[i]->center.y - sq_amt;
                cvLine(m_pImage, pt1, pt2, lineColor, 3, CV_AA, 0);
        }
}

void Camera::DetermineAdjustment( )
{
        bool adjust;
		
        // if 1 square found 
		if ( m_vBiggest.size() == 1 )
        {
                // if square on left side of screen, turn right
                if( m_vBiggest[0]->center.x < 320 )
                        m_iDirection = RI_TURN_RIGHT;
                else
                        m_iDirection = RI_TURN_LEFT;

                // set adjust flag and continue
                adjust = true;
        }

        // if 2 squares found, draw line connecting them
        else if( m_vBiggest.size() == 2 )
        {
                DrawSquareLine( m_vBiggest, &m_dSlope, &m_CvPCenterPoint );
                DrawXOnSquares( m_vBiggest, CV_RGB(255, 0, 0) );
        }

        
        // adjustment required if m_dSlope is outside range or centerPoint is too far off center
        bool isSlopeOutsideRange = m_dSlope >= SLOPE_TOLERANCE || m_dSlope <= -SLOPE_TOLERANCE;
        bool centerOffset = m_CvPCenterPoint.x - 320;
        if ( isSlopeOutsideRange == true || abs(centerOffset) > OFFSET_TOLERANCE )
        {
                if ( isSlopeOutsideRange == true )
                {
                        // left turn required
                        if ( m_dSlope > 0 )
                                m_iDirection = RI_TURN_LEFT;
                        // right turn required
                        else
                                m_iDirection = RI_TURN_RIGHT;
                }
                else
                {
                        // left turn required
                        if ( centerOffset < 0 )
                                m_iDirection = RI_TURN_LEFT;
                        // right turn required
                        else
                                m_iDirection = RI_TURN_RIGHT;
                }

                adjust = true;
        }

        // no squares found
        else
        {
                adjust = false;
        }

        m_bAdjust = adjust;
}