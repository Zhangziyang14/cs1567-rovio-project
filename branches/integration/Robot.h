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
#include "PID.h"
#include "Kalman.h"
#include "Camera.h"
#define NORTH 90     // Degree
#define EAST 180    // Degree
#define SOUTH 270   // Degree
#define WEST 0    // Degree 


class Robot
{
protected:
    int currFacing; //current facing
    int curr_x;      //current X
    int curr_y;      //current Y

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

	Camera *m_camera;
	string m_pRobotName;
    
public:
	Robot(string, int);
	~Robot();
	
    void MoveTo(int nextFacing);
    void TurnTo(float target);
    void Init();
    void ReadData();
	void test();
	void move(int);
    map_obj_t* getMap(int* score_1,int* score_2);
    int reserveMap(int x, int y);
    int updateMap(int x, int y);
	
private:

    void updateNS(int flag);
    void updateWE(int flag);
    void updateKalman();
	void setParameter(string);
    float CorrectTheta(float old, int roomID);
	int WheelAverageX( float rightEncoder, float leftEncoder );
	int WheelAverageY( float rightEncoder, float leftEncoder, float rear );
};

#endif