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
    
public:
	Robot(string);
	~Robot();
	
    void MoveTo(float targetX, float targetY);
    void TurnTo(float target);
    void Init();
    void ReadData();
	void test();
	
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
};

#endif