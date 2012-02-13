/* 
 * R_functions.h
 * functions
 * 
 * cs 1567
 *
*/
#ifndef _r_functions_h
#define _r_functions_h
 

float DegreeFilter(float rawD,int roomID){
	float degree = rawD;
	switch(roomID) {
		case 2: //room 2
			degree = 1.5 - rawD;
			break;
		case 3: //room3
			degree = 3 - rawD;
			break;
		case 4: //room 4
			degree = 4 - rawD;
			break;
		case 5: //room 5
			degree = 5 - rawD;
			break;
		default:
			//printf("room %d ....???",roomID);
			return 0;
	}
	return (float)(((int)degree)%360);
}
 