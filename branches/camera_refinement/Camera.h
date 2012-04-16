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
	const char *m_pRobotName;

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

	void InitCamera( RobotInterface *robot, const char *robotName );
    void CamCenter();
    vector<squares_t *> GetSortedSquares( int *fsmCode );
	void MergeSortSquares( squares_t **unsorted_squares );
	void SplitSquares(squares_t *source, squares_t **frontRef, squares_t **backRef);
	squares_t *MergeSquares( squares_t *a, squares_t *b );
	vector<squares_t *> RemoveDuplicateSquares( vector<squares_t *> squares, int index );
	int DetermineFSMState( vector<squares_t *> );
    void DrawSquareLine( vector<squares_t *> biggest, double *slope, CvPoint *centerPoint );
    void DrawXOnSquares( vector<squares_t *> squares, CvScalar lineColor );
    void DetermineAdjustment( vector<squares_t *> squares );
	IplImage* ConvertImageRGBtoHSV(const IplImage *imageRGB);
	squares_t *FindSquares(IplImage* img, int threshold);
};

#endif