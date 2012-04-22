#include "Game.h"


#define N 0
#define W 270
#define S 180
#define E 90

#define MIN_SEARCH_DEPTH 4

#define IS_VALID_CELL(x,y) ((x)>=0 && (x)<7 && (y)>=0 && (y)<5 \
		&& !((y)%2==1 &&(x)%2==1))

#define COPY_MAP_OBJ_T(o1,o2) {(o1).x=(o2).x;(o1).y=(o2).y;\
			       (o1).type=(o2).type;(o1).points=(o2).points;(o1).next=NULL;}

#define ABS(x) ((x)<0?-(x):(x))


Game::Game(map_obj_t *map, int type)
{
	searchDepth = MIN_SEARCH_DEPTH;	//set search depth to 4
	robotIType = type;
	map_obj_t *p = map;
	bGameOver = false;
	for(int j=0; j<5; j++) {//go through each node in the link list
		for(int i=0; i<7; i++) {
			//copy the value of map_obj_t
			COPY_MAP_OBJ_T(board[i][j].obj,*p);

			if(i%2==1 && j%2==1) {//if it's a post, continue
				continue;
			}
			//decide if it's a robot
			if(p->type == MAP_OBJ_ROBOT_1 || p->type == MAP_OBJ_ROBOT_2) {
				if(p->type == robotIType) {//if that is our robot
					robotI = &board[i][j];
					printf("robotI:[%d][%d]\n",i,j );
				}
				else{	//if that's the enemy robot
					robotU = &board[i][j];
					printf("robotU-:[%d][%d]\n",i,j );
				}
			}
			//link adjacent cells
			if(IS_VALID_CELL(i,j-1)) {//if there is a valid cell above
				board[i][j].NP = &board[i][j-1];
			}
			if(IS_VALID_CELL(i-1,j)) {//if there is a valid cell to the left
				board[i][j].WP = &board[i-1][j];
			}
			if(IS_VALID_CELL(i,j+1)) {//if there is a valid cell below
				board[i][j].SP = &board[i][j+1];
			}
			if(IS_VALID_CELL(i+1,j)) {//if there is a valid cell to the right
				board[i][j].EP = &board[i+1][j];
			}
			p = p->next;//point to next map_obj_t
		}
	}
	printf("Robot1:%d Robot2:%d US:%d\n",MAP_OBJ_ROBOT_1,MAP_OBJ_ROBOT_2,robotIType);
	printf("Initialized GAME\n");
	printMap(map);
}


Game::~Game(void)
{

}


/**
 * update current board and update robotI and robotU
 */
void Game::updateGame(map_obj_t *map, int x, int y)
{
	int minCoinDistance = 100;//set minCoinDistance to a large value
	map_obj_t *p = map;
	for(int j=0; j<5; j++) {//go through each node in the link list
		for(int i=0; i<7; i++) {
			//copy the value of map_obj_t
			COPY_MAP_OBJ_T(board[i][j].obj,*p);
			//decide if it's a robot
			if(p->type == MAP_OBJ_ROBOT_1 || p->type == MAP_OBJ_ROBOT_2) {
				if(p->type == robotIType) //if that is our robot
					robotI = &board[i][j];
				else	//if that's the enemy robot
					robotU = &board[i][j];
			}
			else if(p->type == MAP_OBJ_PELLET) {//if it's a coin
				//distance between the coin and our robot
				int distance = ABS(p->x - x)+ABS(p->y - y);
				if(distance<minCoinDistance)
					minCoinDistance = distance;	//update minCoinDistance
			}
			p = p->next;//point to next map_obj_t
		}
	}
	if(minCoinDistance == 100) {
		bGameOver = true;
		return;
	}
	//if the nearest coin
	if(minCoinDistance > searchDepth) {
		printf("depth change from %d to %d\n",searchDepth, minCoinDistance);
		searchDepth = minCoinDistance;
	}
}


/**
* get next move based on robot's current position
* next move will be updated on nextX and nextY
* @x robot's current x position
* @y robot's current y position
* @return 0 if find a next move, else return -1
*/
int Game::findMove(map_obj_t *map, int x, int y) {
	updateGame(map, x, y);
	return updateNextMove(robotI,0);
}


#define FIND_MAX(node, dir)	if((node)!=NULL) {				\
					updateNextMove((node),depth+1);		\
					if((node)->total >= maxValue) {		\
						maxValue = (node)->total;	\
						rt = dir;			\
					}					\
				}


/**
 * depth first search recursive call
 */
int Game::updateNextMove(GNode *gnode, int depth) {
	//if the other robot is near, dont choose this path
	if(gnode->NP == robotU || gnode->WP == robotU ||
		gnode->SP == robotU || gnode->EP == robotU) {
		gnode->total = -100;
		return 0;
	}
	if(depth == searchDepth) {//last move
		gnode->total = gnode->getPoints();
		return 0;
	}
	int rt = -1;
	int maxValue = 0;
	gnode->bUsed = true;
	FIND_MAX(gnode->NP, N);
	FIND_MAX(gnode->WP, W);
	FIND_MAX(gnode->SP, S);
	FIND_MAX(gnode->EP, E);

	gnode->bUsed = false;
	gnode->total = gnode->getPoints() + maxValue;
	return rt;
}


bool Game::isGameOver() {
	return bGameOver;
}


void Game::printMap(map_obj_t *map) {
	int x = 0;
	map_obj_t *temp = map;

	std::cout << std::endl;

	while(temp != NULL) {
		switch(temp->type) {
			case MAP_OBJ_EMPTY:
				std::cout << "-\t";
				break;	
			case MAP_OBJ_PELLET:
				std::cout << temp->points << "\t";
				break;
			case MAP_OBJ_ROBOT_1:
				if(robotIType==MAP_OBJ_ROBOT_1){
					std::cout << "r\t";}
				else{
					std::cout << "e\t";}
				break;
			case MAP_OBJ_ROBOT_2:
				if(robotIType==MAP_OBJ_ROBOT_2){
					std::cout << "r\t";}
				else{
					std::cout << "e\t";}
				break;
			case MAP_OBJ_POST:
				std::cout << "X\t";
				break;
		}
		
		x++;

		if(x >= MAP_MAX_X) {
			std::cout << std::endl;
			x = 0;
		}

		temp = temp -> next;
	}

	std::cout << std::endl;
}
