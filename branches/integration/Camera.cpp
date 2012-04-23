/** * * * * * * * * * * * * * * * * * * * * *
* Camera.cpp								*
* Created: 3/23/2012						*
* Authors: CJ McAllister, Yuxin Liu         *
*											*
* * * * * * * * * * * * * * * * * * * * * * */

#include <iostream>
#include <fstream>

#include "Camera.h"

#define SLOPE_TOLERANCE	 0.075
#define OFFSET_TOLERANCE 20

#define FSM_NO_SQUARES	0x00
#define FSM_ONE_SQUARE  0x01
#define FSM_PAIR		0x02
#define FSM_NO_PAIR		0x03

using namespace std;


Camera::Camera()
{	
    m_pImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
    m_pThreshold = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);

    m_CvPCenterPoint = cvPoint(0, 0);
}

Camera::~Camera()
{
	cvReleaseImage(&m_pImage);
	cvReleaseImage(&m_pHsv);
	cvReleaseImage(&m_pThreshold);
}

void Camera::InitCamera( RobotInterface *robot, string robotName )
{
	cout << "Initializing robot: " << robotName << endl;
	//set RobotInterface pointer to member variable
	m_robot = robot;
	m_pRobotName = robotName;
		
    // Setup the camera
    if(m_robot->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, RI_CAMERA_DEFAULT_CONTRAST, 5, RI_CAMERA_RES_640, RI_CAMERA_QUALITY_HIGH)) {
        cout << "Failed to configure the camera!" << std::endl;
        exit(-1);
    }
        
    // Create a window to display the output
    cvNamedWindow("Pink Squares", CV_WINDOW_AUTOSIZE);
        
    // Initialize correction data
    m_iDirection = RI_MOVE_FORWARD;
    m_bAdjust = false;
    m_dSlope = 0.0;

    // zero out images
    cvZero(m_pImage);
    cvZero(m_pThreshold);

    // Move the head up to the middle position
    m_robot->Move(RI_HEAD_MIDDLE, RI_FASTEST);
}

/** * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * void CamCenter()											*
  *																*
  * Centers robot in path using squares found in camera input	*
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Camera::CamCenter()
{
	int fsmCode;
	m_bAdjust = true;
	int itr = 0;

	while( m_bAdjust ){
#ifdef DEBUG
		cout << endl << endl << "NEW LOOP" << endl;
#endif
		itr++;

		if ( itr > 25 )
			break;

        if(m_robot->update() != RI_RESP_SUCCESS) {
            std::cout << "Failed to update sensor information!" << std::endl;
            continue;
        }
		
        // Get the current camera m_pImage and display it
        if(m_robot->getImage(m_pImage) != RI_RESP_SUCCESS) {
            std::cout << "Unable to capture an image!" << std::endl;
            continue;
        }

        // Update the UI
        cvWaitKey(10);
        
		// handle biggest squares based on fsmCode
        m_vSquares = GetSortedSquares( &fsmCode );

		switch( fsmCode )
		{
			case FSM_NO_SQUARES:
#ifdef DEBUG
				cout << "FSM_NO_SQUARES" << endl;
#endif
				continue;
			case FSM_ONE_SQUARE:
#ifdef DEBUG
				cout << "FSM_ONE_SQUARE" << endl;
#endif
				m_vBiggest.push_back(m_vSquares[0]);
				break;
			case FSM_PAIR:
#ifdef DEBUG
				cout << "FSM_PAIR" << endl;
#endif
				m_vBiggest.push_back(m_vSquares[0]);
				m_vBiggest.push_back(m_vSquares[1]);
				DrawSquareLine( m_vBiggest );
				break;
			case FSM_NO_PAIR:
#ifdef DEBUG
				cout << "FSM_NO_PAIR" << endl;
#endif
				m_vBiggest.push_back(m_vSquares[0]);
				m_vBiggest.push_back(m_vSquares[1]);
				DrawSquareLine( m_vBiggest );
				break;
			default:
				cout << "FSM error. Exiting" << endl;
				exit(-1);
		}

		// determine if adjust needed based on squares in m_vBiggest
        DetermineAdjustment( m_vBiggest );
		
        // Display the image
        cvLine(m_pImage, cvPoint(320, 480), cvPoint(320, 0), CV_RGB(255, 0, 255), 3);
        cvShowImage("Pink Squares", m_pImage);

#ifdef DEBUG
        cvShowImage("Threshold", m_pThreshold);
#endif

#ifdef DEBUG
        printf("m_dSlope: %.3f\n", m_dSlope);
        printf("centerPoint.x: %d\n", m_CvPCenterPoint.x);
#endif   
		
        if( m_bAdjust )
        {
            cout << "MOVING" << endl;
			m_robot->Move(m_iDirection, 3);
			// only do quick movement on turns
			if ( m_iDirection == RI_TURN_LEFT || m_iDirection == RI_TURN_RIGHT )
				m_robot->Move(RI_STOP, 1);
        }

		// write image to file when exiting loop
		else
		{
#ifdef DEBUG
			cout << "Exiting loop, saving image and data" << endl;
			cvSaveImage("m_pImage.jpg", m_pImage);
			cvSaveImage("m_pThreshold.jpg", m_pThreshold); 

			ofstream finalDataFile;
			finalDataFile.open( "final_data.txt" );
			finalDataFile << "Slope: " << m_dSlope << endl;
			finalDataFile << "Center Point: (" << m_CvPCenterPoint.x << ", " << m_CvPCenterPoint.y << ")" << endl;
			finalDataFile << "Squares:" << endl;
			for ( unsigned int i=0; i<m_vSquares.size(); i++ )
				finalDataFile << i << ": Area: " << m_vSquares[i]->area << " Coords: (" << m_vSquares[i]->center.x << ", " << m_vSquares[i]->center.y << ")" << endl;
			finalDataFile << "Biggest:" << endl;
			for ( unsigned int i=0; i<m_vBiggest.size(); i++ )
				finalDataFile << i << ": Area: " << m_vBiggest[i]->area << " Coords: (" << m_vBiggest[i]->center.x << ", " << m_vBiggest[i]->center.y << ")" << endl;
			finalDataFile.close();
#endif
		}

		// release data from vectors and such
		m_vBiggest.clear();
		m_vSquares.clear();
		cvZero(m_pImage);
		cvZero(m_pHsv);
		cvZero(m_pThreshold);
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

    // Convert the m_pImage from RGB to HSV
	m_pHsv = ConvertImageRGBtoHSV( m_pImage );
        
    IplImage *pink1 = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
    IplImage *pink2 = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
    cvZero(pink1);
    cvZero(pink2);

    // filter and combine for threshold image
	if ( m_pRobotName.compare( "bender" ) == 0 )
	{
		cvInRangeS(m_pHsv, cvScalar(230, 100, 100), cvScalar(256, 200, 256), pink1);
		cvInRangeS(m_pHsv, cvScalar(0, 100, 100), cvScalar(22, 200, 256), pink2);
	}
	else if ( m_pRobotName.compare( "rosie" ) == 0 )
	{
		cvInRangeS(m_pHsv, cvScalar(230, 100, 100), cvScalar(256, 200, 256), pink1);
		cvInRangeS(m_pHsv, cvScalar(0, 100, 100), cvScalar(22, 200, 256), pink2);
	}
	else
	{
		cout << "Invalid robot name: \"" << m_pRobotName << "\", exiting" << endl;
		exit(-1);
	}
    cvOr(pink1, pink2, m_pThreshold);
	cvSmooth(m_pThreshold, m_pThreshold, CV_MEDIAN, 7, 7);


#ifdef DEBUG
	IplImage *pinkSum = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
	IplImage *pinkSumSmooth = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
	IplImage *hue = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
	IplImage *hue1 = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
	IplImage *hue2 = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
	IplImage *hue3 = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
	IplImage *sat = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
	IplImage *sat1 = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
	IplImage *val = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
	IplImage *val1 = cvCreateImage(cvGetSize(m_pHsv), m_pHsv->depth, 1);
    cvZero(pinkSum);
    cvZero(pinkSumSmooth);
    cvZero(hue);
    cvZero(hue1);
    cvZero(hue2);
    cvZero(hue3);
    cvZero(sat);
    cvZero(sat1);
    cvZero(val);
    cvZero(val1);
	
	cvOr(pink1, pink2, pinkSum);
	cvSmooth(pinkSum, pinkSumSmooth, CV_MEDIAN, 7, 7);
	cvSplit(m_pHsv, hue, sat, val, NULL);
	
	cvInRangeS(val, cvScalar(100), cvScalar(256), val1);
	//cvInRangeS(hue, cvScalar(0), cvScalar(22), hue2);
	//cvOr(hue1, hue2, hue3);		
	
    cvNamedWindow("pinkSum", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("pinkSumSmooth", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("val", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("val1", CV_WINDOW_AUTOSIZE);
    //cvNamedWindow("hue1", CV_WINDOW_AUTOSIZE);
    //cvNamedWindow("hue2", CV_WINDOW_AUTOSIZE);
    //cvNamedWindow("hue3", CV_WINDOW_AUTOSIZE);
    cvShowImage("pinkSum", pinkSum);
    cvShowImage("pinkSumSmooth", pinkSumSmooth);
    cvShowImage("val", val);
    cvShowImage("val1", val1);
    //cvShowImage("hue1", hue1);
    //cvShowImage("hue2", hue2);
    //cvShowImage("hue3", hue3);
#endif

    cvReleaseImage(&pink1);
    cvReleaseImage(&pink2);
        
    // Find and sort the squares in the m_pImage, then draw X's
    tmpSquares = FindSquares(m_pThreshold, 200 );
	MergeSortSquares( &tmpSquares );

	// put squares into vector
	while ( tmpSquares != NULL && tmpSquares->next != NULL )
	{
		vSquares.push_back(tmpSquares);
		tmpSquares = tmpSquares->next;
	}

	vSquares = RemoveDuplicateSquares( vSquares, 0 );

	*fsmCode = DetermineFSMState( vSquares );

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

vector<squares_t *> Camera::RemoveDuplicateSquares( vector<squares_t *> squares, unsigned int index )
{
	// base case
	if ( squares.size() - index == 0 )
	{
		return squares;
	}

	else
	{
		CvPoint comparePoint = squares[index]->center;
		
		for ( unsigned int i=0; i<squares.size(); i++ )
		{
			// skip index
			if ( i == index )
				continue;
			
			int xDiff = abs( squares[i]->center.x - comparePoint.x );
			int yDiff = abs( squares[i]->center.y - comparePoint.y );
			
			// check for dup and erase if found
			if ( xDiff <= 5 && yDiff <= 5 )
			{
				squares.erase(squares.begin()+i);

				// reset index to check for other possible dups
				index = 0;
				return RemoveDuplicateSquares(squares, index);
			}
		}

		// decreement and recurse
		index++;
		return RemoveDuplicateSquares(squares, index);
	}
}

int Camera::DetermineFSMState( vector<squares_t *> squares ) 
{
#ifdef DEBUG
	cout << "DetermineFSMState:" << endl;
	for( unsigned int i=0; i<squares.size(); i++ )
	{
		cout << "square" << i << " area: " << squares[i]->area << " coords: (" << squares[i]->center.x << ", " << squares[i]->center.y << ")" << endl;
	}
#endif

	// no squares found
	if ( squares.size() == 0 )
		return FSM_NO_SQUARES;

	// 1 square found
	if (squares.size() == 1 )
		return FSM_ONE_SQUARE;

	// check for pair
	for( unsigned int i=0; i<squares.size(); i++ )
	{
		for ( unsigned int j=0; j<squares.size(); j++ )
		{
			// avoid self compares
			if ( j == i )
				continue;

			int yDiff = abs( squares[i]->center.y - squares[j]->center.y );
			if ( yDiff <= 15 )
				return FSM_PAIR;
		}
	}

	// 2+ squares, but not paired
	return FSM_NO_PAIR;
}

/**
 * Draws a line between the 2 biggest squares
 * 
 * Returns slope and centerPoint output params
 */
void Camera::DrawSquareLine( vector<squares_t *> biggest )
{
    CvPoint pt1, pt2;

    int xCenter = (biggest[0]->center.x + biggest[1]->center.x)/2;
    int yCenter = (biggest[0]->center.y + biggest[1]->center.y)/2;
	
	// draw slope tolerance shape
	for ( float i = -SLOPE_TOLERANCE; i < SLOPE_TOLERANCE; i = i+(SLOPE_TOLERANCE/5) )
	{
		pt1.x = min( biggest[0]->center.x, biggest[1]->center.x );
		pt1.y = yCenter - ( 320 * i );
		pt2.x = max( biggest[0]->center.x, biggest[1]->center.x );
		pt2.y = yCenter + ( 320 * i );
		cvLine( m_pImage, pt1, pt2, CV_RGB(127, 127, 127), 3 );
	}

    // draw vertal line  between squares
    pt1.x = xCenter;
    pt1.y = yCenter + 10;
    pt2.x = xCenter;
    pt2.y = yCenter - 10;
                
    cvLine( m_pImage, pt1, pt2, CV_RGB(0, 0, 127), 3 );

	// draw vertical lines at offset tolerance points	
	pt1.x = xCenter - OFFSET_TOLERANCE;
	pt2.x = xCenter - OFFSET_TOLERANCE;
	pt1.y = yCenter + 25;
	pt2.y = yCenter - 25;
	cvLine( m_pImage, pt1, pt2, CV_RGB( 0, 127, 127 ), 3 );
	
	pt1.x = xCenter + OFFSET_TOLERANCE;
	pt2.x = xCenter + OFFSET_TOLERANCE;
	cvLine( m_pImage, pt1, pt2, CV_RGB( 0, 127, 127 ), 3 );

	// draw horizontal line connecting squares        
    pt1.x = biggest[0]->center.x;
    pt1.y = biggest[0]->center.y;
    pt2.x = biggest[1]->center.x;
    pt2.y = biggest[1]->center.y;

    cvLine( m_pImage, pt1, pt2, CV_RGB(0, 0, 255), 3 );

    m_dSlope = -((double)pt2.y - (double)pt1.y) / ((double)pt2.x - (double)pt1.x);
    m_CvPCenterPoint = cvPoint(xCenter, yCenter);
}

void Camera::DrawXOnSquares( vector<squares_t *> squares, CvScalar lineColor )
{
        CvPoint pt1, pt2;
        int sq_amt;

        // draw until end of list reached
        for( unsigned int i=0; i<squares.size(); i++ ) {
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

void Camera::DetermineAdjustment( vector<squares_t *> squares )
{
	cout << "DetermineAdjustment:" << endl;
	cout << "squares.size(): " << squares.size() << endl;
    bool adjust;
		
    // if 1 square found 
	if ( squares.size() == 1 )
    {
        // if square on left side of screen, turn left
        if( squares[0]->center.x < 320 )
            m_iDirection = RI_TURN_LEFT;
        else
            m_iDirection = RI_TURN_RIGHT;

        // set adjust flag and continue
        adjust = true;
    }

    // if 2 squares found, draw line connecting them
    else if( squares.size() == 2 )
    {
        DrawXOnSquares( squares, CV_RGB(255, 0, 0) );
        
		// adjustment required if m_dSlope is outside range or centerPoint is too far off center
		bool isSlopeOutsideRange = m_dSlope >= SLOPE_TOLERANCE || m_dSlope <= -SLOPE_TOLERANCE;
		int centerOffset = m_CvPCenterPoint.x - 320;
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
					m_iDirection = RI_MOVE_LEFT;
				// right turn required
				else
					m_iDirection = RI_MOVE_RIGHT;
			}

			adjust = true;
		}
		else
		{
			cout << "Within margin, stopping adjustment" << endl;
			adjust = false;
		}
	}

    // no squares found, turn opposite of previous turn
    else
    {
		adjust = true;
		if( m_iDirection == RI_TURN_RIGHT )
			m_iDirection = RI_TURN_LEFT;
		else
			m_iDirection = RI_TURN_RIGHT;
    }

    m_bAdjust = adjust;
}

/**
 * Original author: Shervin Emami
 * Citation: http://shervinemami.co.cc/colorConversion.html
 */
IplImage* Camera::ConvertImageRGBtoHSV(const IplImage *imageRGB)
{
	float fR, fG, fB;
	float fH, fS, fV;
	const float FLOAT_TO_BYTE = 255.0f;
	const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;

	// Create a blank HSV image
	IplImage *imageHSV = cvCreateImage(cvGetSize(imageRGB), 8, 3);
	if (!imageHSV || imageRGB->depth != 8 || imageRGB->nChannels != 3) {
		printf("ERROR in convertImageRGBtoHSV()! Bad input image.\n");
		exit(1);
	}

	int h = imageRGB->height;		// Pixel height.
	int w = imageRGB->width;		// Pixel width.
	int rowSizeRGB = imageRGB->widthStep;	// Size of row in bytes, including extra padding.
	char *imRGB = imageRGB->imageData;	// Pointer to the start of the image pixels.
	int rowSizeHSV = imageHSV->widthStep;	// Size of row in bytes, including extra padding.
	char *imHSV = imageHSV->imageData;	// Pointer to the start of the image pixels.
	for (int y=0; y<h; y++) {
		for (int x=0; x<w; x++) {
			// Get the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
			uchar *pRGB = (uchar*)(imRGB + y*rowSizeRGB + x*3);
			int bB = *(uchar*)(pRGB+0);	// Blue component
			int bG = *(uchar*)(pRGB+1);	// Green component
			int bR = *(uchar*)(pRGB+2);	// Red component

			// Convert from 8-bit integers to floats.
			fR = bR * BYTE_TO_FLOAT;
			fG = bG * BYTE_TO_FLOAT;
			fB = bB * BYTE_TO_FLOAT;

			// Convert from RGB to HSV, using float ranges 0.0 to 1.0.
			float fDelta;
			float fMin, fMax;
			int iMax;
			// Get the min and max, but use integer comparisons for slight speedup.
			if (bB < bG) {
				if (bB < bR) {
					fMin = fB;
					if (bR > bG) {
						iMax = bR;
						fMax = fR;
					}
					else {
						iMax = bG;
						fMax = fG;
					}
				}
				else {
					fMin = fR;
					fMax = fG;
					iMax = bG;
				}
			}
			else {
				if (bG < bR) {
					fMin = fG;
					if (bB > bR) {
						fMax = fB;
						iMax = bB;
					}
					else {
						fMax = fR;
						iMax = bR;
					}
				}
				else {
					fMin = fR;
					fMax = fB;
					iMax = bB;
				}
			}
			fDelta = fMax - fMin;
			fV = fMax;				// Value (Brightness).
			if (iMax != 0) {			// Make sure its not pure black.
				fS = fDelta / fMax;		// Saturation.
				float ANGLE_TO_UNIT = 1.0f / (6.0f * fDelta);	// Make the Hues between 0.0 to 1.0 instead of 6.0
				if (iMax == bR) {		// between yellow and magenta.
					fH = (fG - fB) * ANGLE_TO_UNIT;
				}
				else if (iMax == bG) {		// between cyan and yellow.
					fH = (2.0f/6.0f) + ( fB - fR ) * ANGLE_TO_UNIT;
				}
				else {				// between magenta and cyan.
					fH = (4.0f/6.0f) + ( fR - fG ) * ANGLE_TO_UNIT;
				}
				// Wrap outlier Hues around the circle.
				if (fH < 0.0f)
					fH += 1.0f;
				if (fH >= 1.0f)
					fH -= 1.0f;
			}
			else {
				// color is pure Black.
				fS = 0;
				fH = 0;	// undefined hue
			}

			// Convert from floats to 8-bit integers.
			int bH = (int)(0.5f + fH * 255.0f);
			int bS = (int)(0.5f + fS * 255.0f);
			int bV = (int)(0.5f + fV * 255.0f);

			// Clip the values to make sure it fits within the 8bits.
			if (bH > 255)
				bH = 255;
			if (bH < 0)
				bH = 0;
			if (bS > 255)
				bS = 255;
			if (bS < 0)
				bS = 0;
			if (bV > 255)
				bV = 255;
			if (bV < 0)
				bV = 0;

			// Set the HSV pixel components.
			uchar *pHSV = (uchar*)(imHSV + y*rowSizeHSV + x*3);
			*(pHSV+0) = bH;		// H component
			*(pHSV+1) = bS;		// S component
			*(pHSV+2) = bV;		// V component
		}
	}
	return imageHSV;
}

squares_t *Camera::FindSquares(IplImage* img, int threshold)
{
	CvSeq* contours;
	CvMemStorage *storage;
	int i, j, area;
	CvPoint ul, lr, pt, centroid;
	CvSize sz = cvSize( img->width, img->height);
	IplImage * canny = cvCreateImage(sz, 8, 1);
	squares_t *sq_head, *sq, *sq_last;
    	CvSeqReader reader;
    
	// Create storage
	storage = cvCreateMemStorage(0);

	CvSeq* result;
	double s, t;

	// Create an empty sequence that will contain the square's vertices
	CvSeq* squares = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvPoint), storage);
    
	// Select the maximum ROI in the image with the width and height divisible by 2
	cvSetImageROI(img, cvRect(0, 0, sz.width, sz.height));

	// Apply the canny edge detector and set the lower to 0 (which forces edges merging) 
	cvCanny(img, canny, 0, 50, 3);
		
	// Dilate canny output to remove potential holes between edge segments 
	cvDilate(canny, canny, 0, 2);

#ifdef DEBUG_SQUARE_CANNY
	cvShowImage("Debug - CANNY", canny);
#endif
		
	// Find the contours and store them all as a list
	cvFindContours(canny, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
            
	// Test each contour to find squares
	while(contours) {
		// Approximate a contour with accuracy proportional to the contour perimeter
		result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.10, 0 );
                // Square contours should have
		//	* 4 vertices after approximation
		// 	* Relatively large area (to filter out noisy contours)
		// 	* Ne convex.
		// Note: absolute value of an area is used because
		// area may be positive or negative - in accordance with the
		// contour orientation
                if(result->total == 4 && fabs(cvContourArea(result,CV_WHOLE_SEQ,0)) > threshold && cvCheckContourConvexity(result)) {
			s=0;
                    	for(i=0; i<5; i++) {
                        	// Find the minimum angle between joint edges (maximum of cosine)
				if(i >= 2) {
					t = fabs(ri_angle((CvPoint*)cvGetSeqElem(result, i),(CvPoint*)cvGetSeqElem(result, i-2),(CvPoint*)cvGetSeqElem( result, i-1 )));
					s = s > t ? s : t;
        	                }
			}
                    
			// If cosines of all angles are small (all angles are ~90 degree) then write the vertices to the sequence 
			if( s < 0.2 ) {
				for( i = 0; i < 4; i++ ) {
					cvSeqPush(squares, (CvPoint*)cvGetSeqElem(result, i));
				}
			}
                }
                
                // Get the next contour
		contours = contours->h_next;
	}

    	// initialize reader of the sequence
	cvStartReadSeq(squares, &reader, 0);
	sq_head = NULL; sq_last = NULL; sq = NULL;
	// Now, we have a list of contours that are squares, find the centroids and area
	for(i=0; i<squares->total; i+=4) {
		// Find the upper left and lower right coordinates
		ul.x = 1000; ul.y = 1000; lr.x = 0; lr.y = 0;
		for(j=0; j<4; j++) {
			CV_READ_SEQ_ELEM(pt, reader);
			// Upper Left
			if(pt.x < ul.x)
				ul.x = pt.x;
			if(pt.y < ul.y)
				ul.y = pt.y;
			// Lower right
			if(pt.x > lr.x)
				lr.x = pt.x;
			if(pt.y > lr.y)
				lr.y = pt.y;
		}

		// Find the centroid
		centroid.x = ((lr.x - ul.x) / 2) + ul.x;
		centroid.y = ((lr.y - ul.y) / 2) + ul.y;

		// Find the area
		area = (lr.x - ul.x) * (lr.y - ul.y);

		// Add it to the storage
		sq = new squares_t;
		// Fill in the data
		sq->area = area;
		sq->center.x = centroid.x;
		sq->center.y = centroid.y;
		sq->next = NULL;
		if(sq_last == NULL) 
			sq_head = sq;	
		else 
			sq_last->next = sq;
		sq_last = sq;
	}	
    
	// Release the temporary images and data
	cvReleaseImage(&canny);
	cvReleaseMemStorage(&storage);
	return sq_head;
}