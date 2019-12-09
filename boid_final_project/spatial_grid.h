#pragma once
#include "boid.h"
#include <vector>
#include <list>
#include <math.h>

class SpatialGrid
{
public:
	SpatialGrid(vector<Boid> &boids);
	~SpatialGrid();
	void UpdateNearCells(Boid &boid);
	bool UpdateGrid(Boid &boid, vector<int> &update_tracker);
	void UpdateGrid(Boid &boid, int old_pos, int new_pos);

private:

	int cell_num;
	float cell_length;
	vector<list<Boid*>> grid;

	int GetGridVectorIndex(vector<int> &grid_index);
	int GetGridVectorIndex(int &x, int &y, int &z);

	vector<int> GetGridIndex(Boid &boid);
	vector<int> GetGridIndex(int &vector_index);
	void AddBoid(Boid &boid);

	
};

