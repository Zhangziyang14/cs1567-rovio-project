#ifndef _ROBOT_H_
#define _ROBOT_H_

/** * * * * * * * * * * * * * * * * * * * * *
* Robot.h                                   *
* Created: 3/12/2012						*
* Authors: CJ McAllister, Yuxin Liu         *
*											*
* * * * * * * * * * * * * * * * * * * * * * */

#include <iostream>

#include "robot_if++.h"
#include "robot_color.h"
#include "Fir.h"
#include "Kalman.h"
#include "PID.h"

class Robot
{
protected:
    Fir *xFilter;
    Fir *yFilter;
    Fir *rightFilter;
    Fir *leftFilter;
    Fir *rearFilter;
    Kalman *kf;
    RobotInterface *robot;
    
    
    float currWX;            //current X position using wheel encoder
	float currWY;            //current Y position using wheel encoder
	float currWTheta;        //current theta using wheel encoder
    
    float currNSX;            // The current North Star X-coordinate reading.
	float currNSY;            // The current North Star Y-coordinate reading.
	float currNSTheta;        // The current North Star Theta reading.
    
    float finalX;
    float finalY;
    float finalTheta;
    
    int roomID;
	
	IplImage *m_pImage;
	IplImage *m_pHsv;
	IplImage *m_pThreshold;
	IplImage *m_pHue;
	IplImage *m_pSat;
	IplImage *m_pVal;

	CvPoint m_CvPpath_center;
    
public:
	Robot(string);
	~Robot();
	
    void MoveTo(float targetX, float targetY);
    void TurnTo(float target);
    void Init();
    void ReadData();
	void InitCamera();
	void CamNav();
	squares_t *FindSquares( int color );
	squares_t *GetBiggestSquares( squares_t *squares );
	double DrawCenterLine( squares_t *squares );
	void DrawOnSquares( squares_t *squares, CvScalar lineColor );

	
private:
    void NS_Rotate(int room);
    void NS_Scale();
    void NS_Align(int room);
    void updateNS(int flag);
    void updateWE(int flag);
    void updateKalman();
    float CorrectTheta(float old, int roomID);
	float WheelAverageX( float rightEncoder, float leftEncoder );
	float WheelAverageY( float rightEncoder, float leftEncoder );
	int ListLength( squares_t *list );
};

#endif