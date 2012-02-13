float xOrigin, yOrigin, rightWheelOrigin, leftWheelOrigin, rearWheelOrigin;

filter *xFilter = NULL;
filter *yFilter = NULL;
filter *rightWheelFilter = NULL;
filter *leftWheelFilter = NULL;
filter *rearWheelFilter = NULL;

RobotInterface *robot;

int InitializeFirFilters( RobotInterface *robot );
int SetOrigin();
int TurnTo( int theta );
int MoveTo( int x, int y );
