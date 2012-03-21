#ifndef _ROBOT_H_
#define _ROBOT_H_

/** * * * * * * * * * * * * * * * * * * * * *
* Robot.h                                   *
* Created: 3/12/2012						*
* Authors: CJ McAllister, Yuxin Liu         *
*											*
* * * * * * * * * * * * * * * * * * * * * * */

#include "robot_if++.h"
#include "Fir.h"
#include "robot_color.h"
#include "FirTheta.h"
#include "Kalman.h"

class Robot
{
protected:
    Fir *xFilter;
    Fir *yFilter;
    Fir *rightFilter;
    Fir *leftFilter;
    Fir *rearFilter;
	FirTheta *thetaFilter;
    Kalman *kf;
    RobotInterface *robot;
    
    
    int currWX;            //current X position using wheel encoder
	int currWY;            //current Y position using wheel encoder
	float currWTheta;        //current theta using wheel encoder
    
    int currNSX;            // The current North Star X-coordinate reading.
	int currNSY;            // The current North Star Y-coordinate reading.
	float currNSTheta;        // The current North Star Theta reading.
    int   currNSD;


	int prevNSX;            // The prev North Star X-coordinate reading.
	int prevNSY;            // The prev North Star Y-coordinate reading.
	float prevNSTheta;        // The prev North Star Theta reading.
	int   prevNSD;

	int wheelR;
	int wheelL;
	int wheelB;


    int finalX;
    int finalY;
    float finalTheta;
    int finalD;

    int roomID;


	IplImage *m_pImage;
    IplImage *m_pHsv;
    IplImage *m_pThreshold;
    IplImage *m_pHue;
    IplImage *m_pSat;
    IplImage *m_pVal;

    CvPoint m_CvPpath_center;

    int m_iDirection;
    bool m_bAdjust;
    double m_dSlope;
    CvPoint m_CvPCenterPoint;
    squares_t *m_pBiggest;
    
public:
	Robot(string);
	~Robot();
	
    void MoveTo(float targetX, float targetY);
    void TurnTo(float target);
    void Init();
    void ReadData();
	void test();
	void move(int);

	void InitCamera();
    void CamCenter();
    squares_t *FindSquares( int color );
    squares_t *GetBiggestPair( squares_t *squares );
    squares_t *GetBiggestSquares( squares_t *squares );
    void DrawSquareLine( squares_t *squares, double *slope, CvPoint *centerPoint );
    void DrawOnSquares( squares_t *squares, CvScalar lineColor );
    bool DetermineAdjustment( squares_t *squares );
	
private:
    void NS_Rotate(int room);
    void NS_Scale();
    void NS_Align(int room);
    void updateNS(int flag);
    void updateWE(int flag);
    void updateKalman();
    float CorrectTheta(float old, int roomID);
	int WheelAverageX( float rightEncoder, float leftEncoder );
	int WheelAverageY( float rightEncoder, float leftEncoder, float rear );

	int ListLength( squares_t *list );
};

#endif