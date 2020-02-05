#pragma once
#include "boid.h"
#include <vector>
#include <list>
#include <cmath>


/**
 * \brief  Spatial data structure for keeping track of boids and quickly working out a given boids neighbours 
 */
class SpatialGrid
{
public:
	SpatialGrid(vector<Boid> &boids);
	~SpatialGrid() = default;

	void UpdateNearCells(Boid &boid);
	bool UpdateGrid(Boid &boid, vector<int> &update_tracker, int &size);
	void UpdateGrid(Boid &boid, int old_pos, int new_pos);

private:

	int cell_num;
	float cell_length;
	vector<list<Boid*>> grid; //Grid holds pointers to boids not boid itself to reduce memory and speed up access.
							  //Each cell corresponds to doubly linked list for quick insertion/removal

	int GetGridVectorIndex(vector<int> &grid_index) const;
	int GetGridVectorIndex(int &x, int &y, int &z) const;

	vector<int> GetGridCoord(Boid &boid);
	vector<int> GetGridCoord(int &vector_index);

	void AddBoid(Boid &boid);

	
};

