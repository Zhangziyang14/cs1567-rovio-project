/** * * * * * * * * * * * * * * * * * * * * *
* Camera.cpp								*
* Created: 3/23/2012						*
* Authors: CJ McAllister, Yuxin Liu         *
*											*
* * * * * * * * * * * * * * * * * * * * * * */

#include <iostream>
#include <fstream>

#include "Camera.h"

#define SLOPE_TOLERANCE 0.1
#define OFFSET_TOLERANCE 30

using namespace std;


Camera::Camera()
{
	m_robot;

    m_pImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
    m_pHsv = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
    m_pThreshold = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);

    m_CvPpath_center = cvPoint(0, 0);

    m_CvPCenterPoint = cvPoint(0, 0);
    m_pBiggest = (squares_t *)malloc(2 * sizeof(squares_t));
}

Camera::~Camera()
{

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
#endif
        
    // Initialize correction data
    m_iDirection = RI_MOVE_FORWARD;
    m_bAdjust = false;
    m_dSlope = 0.0;
    squares_t *m_pBiggest = NULL;

    // zero out images
    cvZero(m_pImage);
    cvZero(m_pHsv);
    cvZero(m_pThreshold);

    // Move the head up to the middle position
    m_robot->Move(RI_HEAD_MIDDLE, RI_FASTEST);
}

void Camera::CamCenter()
{
        if( m_bAdjust )
        {
                m_robot->Move(m_iDirection, 3);
                m_robot->Move(RI_STOP, 1);
        }

        if(m_robot->update() != RI_RESP_SUCCESS) {
                std::cout << "Failed to update sensor information!" << std::endl;
                return;
        }

        // Get the current camera m_pImage and display it
        if(m_robot->getImage(m_pImage) != RI_RESP_SUCCESS) {
                std::cout << "Unable to capture an image!" << std::endl;
                return;
        }

        // construct filtered m_pImage
        squares_t *pinkSquares = FindSquares( RC_PINK );
        //squares_t *yelSquares = FindSquares( RC_YELLOW );
                
        m_pBiggest = GetBiggestPair( pinkSquares );

        // if no pairs found
        if ( m_pBiggest == NULL )
                m_pBiggest = GetBiggestSquares( pinkSquares );

        m_bAdjust = DetermineAdjustment( m_pBiggest );

        // draw line down image center
        cvLine(m_pImage, cvPoint(320, 480), cvPoint(320, 0), CV_RGB(255, 0, 255), 3);

        // Display the image(s)
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

        // Release the square and image data
        squares_t *sq_tmp;
        while(pinkSquares != NULL) {
                sq_tmp = pinkSquares->next;
                delete(pinkSquares);
                pinkSquares = sq_tmp;
        }

        while( m_pBiggest != NULL)
        {
                sq_tmp = m_pBiggest->next;
                delete(m_pBiggest);
                m_pBiggest = sq_tmp;
        }
}

squares_t *Camera::FindSquares( int color )
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
        squares = m_robot->findSquares(m_pThreshold, 300);

        DrawOnSquares(squares, lineColor);

        return squares;
}

squares_t *Camera::GetBiggestPair( squares_t *squares)
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

squares_t *Camera::GetBiggestSquares( squares_t *squares )
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
void Camera::DrawSquareLine( squares_t *biggest, double *slope, CvPoint *centerPoint )
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

void Camera::DrawOnSquares( squares_t *squares, CvScalar lineColor )
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

bool Camera::DetermineAdjustment( squares_t *squares )
{
        bool adjust;

        // if 1 square found 
        if ( m_pBiggest != NULL && m_pBiggest->next == NULL )
        {
                // if square on left side of screen, turn right
                if( m_pBiggest->center.x < 320 )
                        m_iDirection = RI_TURN_RIGHT;
                else
                        m_iDirection = RI_TURN_LEFT;

                // set adjust flag and continue
                adjust = true;
        }

        // if 2 squares found, draw line connecting them
        else if( m_pBiggest != NULL && m_pBiggest->next != NULL )
        {
                DrawSquareLine( m_pBiggest, &m_dSlope, &m_CvPCenterPoint );
                DrawOnSquares( m_pBiggest, CV_RGB(255, 0, 0) );
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

        return adjust;
}