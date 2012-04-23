/** * * * * * * * * * * * * * * * * * * * * *
 * test.cpp									*
 * Created: 3/12/2012						*
 * Authors: CJ McAllister, Yuxin Liu         *
 *											*
 * * * * * * * * * * * * * * * * * * * * * * */
 
#include "robot_if++.h"
#include "Game.h"
#include "Robot.h"
#include "stdio.h"
#include "string"
#include "ostream"

using namespace std;
 

int main(int argv, char **argc) {
 	int score_1, score_2, curr_x, curr_y;
	int facing, major_version, minor_version, robot_number;
	map_obj_t *map, *temp;
	RobotInterface *robotObj;
	Game *game_node;
	Robot *robot;

	// Make sure we have a valid command line argument.
	if(argv <= 2) {
		std::cout << "Usage: ./run_game <address of robot> <game ID of robot>" << std::endl;
		exit(-1);
	}

	// Set the start position.  If the second command line argument is
	// 2, then we are robot 2.
	if(atoi(argc[2]) == 2) {
		curr_x = 6;
		curr_y = 2;
		robot_number = MAP_OBJ_ROBOT_2;
	}
	// If the second command line argument is 1, then we are robot 1.
	else if(atoi(argc[2]) == 1){
		curr_x = 0;
		curr_y = 2;
		robot_number = MAP_OBJ_ROBOT_1; 
	}
	else {
		std::cout << "Usage: ./run_game <address of robot> <game ID of robot>" << std::endl;
		std::cout << "The robot game ID can only be 1 or 2" << std::endl;
		//exit(-1);
	}
	std::cout << "We are robot " << robot_number << " starting at (" << curr_x << "," << curr_y;
	std::cout << ")" << std::endl;


/****************************** START *****************************************/
	robot = new Robot(argc[1],atoi(argc[2]));
	robot->Init(); 
	//robot->ReadData();
	//robot -> MoveTo(NORTH);
	//robot->TurnTo(SOUTH);
	
	//exit(0);

	game_node  = new Game(robot->getMap(&score_1, &score_2), robot_number);
	//robot -> MoveTo(NORTH);
	//robot -> MoveTo(NORTH);
	//robot -> MoveTo(SOUTH);
	//robot -> MoveTo(WEST);
	//robot -> MoveTo(EAST);

	printf("done.\n");
	
	
	do {
		// Update the server with the robot's corrent position.
		robot -> updateMap(curr_x, curr_y);


		// Get the new map.
		map = robot -> getMap(&score_1, &score_2);
		std::cout << "Score: " << score_1 << " to " << score_2 << std::endl;


		game_node -> printMap(map);
		

		// Get the best move.  If -2 is returned, then we are waiting for the
		// other robot to move.
		do {
			facing = game_node -> findMove(map, curr_x, curr_y);
		} while (facing == -2);

		
		// Set the new X and Y. 
		switch(facing) {
			case NORTH:
				curr_y--;break;
			case WEST:
				curr_x--;break;
			case SOUTH:
				curr_y++;break;
			case EAST:
				curr_x++;break;
		}
		
		
		robot->reserveMap(curr_x,curr_y);
		printf("Reserved %d,%d\n", curr_x,curr_y);
		
		
		// Are there points left on the map?
		if(facing != -1) {
			// robot move here
			std::cout << "Moving to " << curr_x << ", " << curr_y << std::endl;

			robot -> MoveTo(facing);
			printf("Facing: %d\n", facing);
			printf("Moved to %d,%d\n", curr_x,curr_y);
		}
		

		// Release the map.
		while(map != NULL) {
			temp = map->next;
			delete(map);
			map = temp;
		}
		
		
		usleep(5000000);
	} while(!game_node->isGameOver());


	// Clean up.
	



	return 0;
}
