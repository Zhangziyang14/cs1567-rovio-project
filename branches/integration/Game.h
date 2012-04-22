

#ifndef _GAME_H_
#define _GAME_H_

#include "robot_if++.h"
#include <iostream>
#include <string>
#include <stdlib.h>
#include "stdio.h"

typedef struct _GRAPH_NODE_T {
	map_obj_t obj;
	int total;
	bool bUsed;
	_GRAPH_NODE_T *NP, *EP, *SP, *WP;//all adjacent cells
	_GRAPH_NODE_T() {
		total = 0;
		bUsed = false;
		NP = NULL;
		EP = NULL;
		SP = NULL;
		WP = NULL;
	}
	int getPoints() {
		if(!bUsed && obj.type == MAP_OBJ_PELLET)
			return obj.points;
		else
			return 0;
	}
}GNode;

class Game
{
private:
	GNode board[7][5];
	GNode *robotI, *robotU;
	int robotIType;//our robot type
	int searchDepth;
	bool bGameOver;
public:
	Game(map_obj_t *, int );
	~Game(void);
	int findMove(map_obj_t *, int , int );
	bool isGameOver();
	void printMap(map_obj_t *);
private:
	void updateGame(map_obj_t *, int x, int y);
	int updateNextMove(GNode *, int );
	int getPoints();
	
};

#endif
