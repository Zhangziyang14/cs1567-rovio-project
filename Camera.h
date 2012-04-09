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
#include <vector>

using namespace std;

class Camera
{
protected:
	RobotInterface *m_robot;

	IplImage *m_pImage;
    IplImage *m_pHsv;
    IplImage *m_pThreshold;

    int m_iDirection;
    bool m_bAdjust;
    double m_dSlope;
    CvPoint m_CvPCenterPoint;
    vector<squares_t *> m_vBiggest;
	vector<squares_t *> m_vSquares;

public:
	Camera();
	~Camera();

	void InitCamera( RobotInterface *robot );
    void CamCenter();
    vector<squares_t *> GetSortedSquares( int *fsmCode );
	void MergeSortSquares( squares_t **unsorted_squares );
	void SplitSquares(squares_t *source, squares_t **frontRef, squares_t **backRef);
	squares_t *MergeSquares( squares_t *a, squares_t *b );
	int DetermineFSMState( squares_t *squares );
    void DrawSquareLine( vector<squares_t *> biggest, double *slope, CvPoint *centerPoint );
    void DrawXOnSquares( vector<squares_t *> squares, CvScalar lineColor );
    void DetermineAdjustment( );
};

#endif