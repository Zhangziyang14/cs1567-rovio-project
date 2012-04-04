#ifndef _CAMERA_H_
#define _CAMERA_H_

/** * * * * * * * * * * * * * * * * * * * * *
* Camera.h                                  *
* Created: 3/23/2012						*
* Authors: CJ McAllister, Yuxin Liu         *
*											*
* * * * * * * * * * * * * * * * * * * * * * */

#include "robot_if++.h"
#include "robot_color.h"

class Camera
{
protected:
	RobotInterface *m_robot;

	IplImage *m_pImage;
    IplImage *m_pHsv;
    IplImage *m_pThreshold;

    CvPoint m_CvPpath_center;

    int m_iDirection;
    bool m_bAdjust;
    double m_dSlope;
    CvPoint m_CvPCenterPoint;
    squares_t *m_pBiggest;
	squares_t *m_pSquares;

public:
	Camera();
	~Camera();

	void InitCamera( RobotInterface *robot );
    void CamCenter();
    int GetSortedSquares( squares_t *squares_found );
	void MergeSortSquares( squares_t **unsorted_squares );
	void SplitSquares(squares_t *source, squares_t **frontRef, squares_t **backRef);
	squares_t *MergeSquares( squares_t *a, squares_t *b );
	int DetermineFSMState( squares_t *squares );
    void DrawSquareLine( squares_t *squares, double *slope, CvPoint *centerPoint );
    void DrawOnSquares( squares_t *squares, CvScalar lineColor );
    bool DetermineAdjustment( squares_t *squares );
};

#endif