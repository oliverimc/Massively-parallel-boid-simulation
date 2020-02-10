#include "pch.h"
#include "spatial_grid.h"
#include <iostream>

/*! \file spatial_grid.cpp
	\brief Implementation of the Spatial Grid class


*/

/**
 * \brief   Creates a grid for given simulation details and adds all boids to appropriate cells
 * \param  boids | Boids to add to grid 
 */
SpatialGrid::SpatialGrid(vector<Boid> &boids)
{
	cell_num = floor(LENGTH / SIGHT_RANGE); // number & size of cells calculated off seeing distance so 27 adjacent will always contain all boids within range
	cell_length = float(LENGTH) / float(cell_num);
	grid.resize(cell_num*cell_num*cell_num);

	for (Boid &boid : boids)
	{
		AddBoid(boid);
	}
}


/**
 * \brief  Updates the 27 cells around the boid so it can query them for nearby boids
 * \param  boid | The boid to update
 */
void SpatialGrid::UpdateNearCells(Boid & boid)
{
	vector<int> boid_grid_coord = boid.GetGridCoord();
	int i = 0; 


	for (int x = -1; x < 2; x++)
	{
		for (int y = -1; y < 2; y++)
		{
			for (int z = -1; z < 2; z++)
			{
				int row_x = boid_grid_coord[0] + x;
				int row_y = boid_grid_coord[1] + y;
				int row_z = boid_grid_coord[2] + z;

				row_x = row_x < cell_num ? row_x : 0;
				row_y = row_y < cell_num ? row_y : 0;
				row_z = row_z < cell_num ? row_z : 0;

				row_x = row_x > -1 ? row_x : cell_num - 1;
				row_y = row_y > -1 ? row_y : cell_num - 1;
				row_z = row_z > -1 ? row_z : cell_num - 1;

				
				boid.neighbouring_cells_buffer_[i] = &grid[GetGridVectorIndex(row_x, row_y, row_z)];
				i++;

			}
		}
	}


}

/**
 * \brief   Checks if boid has moved grid cells and if so moves its pointer and stores track of changes
 *			Alters behaviour on multi-node system to avoid double adding boid when it switches cell.
 * \param  boid | Boid to update in grid
 * \param  update_tracker | Vector that stores update information for syncing across nodes
 * \param  size | Number of nodes of system, to determine what routine to run.
 * \return  | Boolean indicating if the boid has moved grid cells
 */
bool SpatialGrid::UpdateGrid(Boid & boid, vector<int>& update_tracker, int &size)
{
	vector<int> old_grid_coord = boid.GetGridCoord();
	vector<int> new_grid_coord = GetGridCoord(boid);

	if (old_grid_coord != new_grid_coord && size == 1 )
	{
		int old_vector_index = GetGridVectorIndex(old_grid_coord);
		int new_vector_index = GetGridVectorIndex(new_grid_coord);

		grid[old_vector_index].remove(&boid);
		grid[new_vector_index].push_back(&boid);
		
		boid.SetGridCoord(new_grid_coord);
		
		
		return true;
	}

	else if (old_grid_coord != new_grid_coord && size != 1)
	{
		int old_vector_index = GetGridVectorIndex(old_grid_coord);
		int new_vector_index = GetGridVectorIndex(new_grid_coord);

		update_tracker.push_back(old_vector_index);
		update_tracker.push_back(new_vector_index);

		return true;
	}
	
	else
	{
		return false;
	}
}

/**
 * \brief  Explicitly updates grid to reflect changes from other nodes 
 * \param  boid | Boid to update.
 * \param  old_vector_index | Vector index of the boids old cell. 
 * \param  new_vector_index | Vector index of the boids new cell
 */
void SpatialGrid::UpdateGrid(Boid & boid, int old_vector_index, int new_vector_index)
{
	grid[old_vector_index].remove(&boid);
	grid[new_vector_index].push_back(&boid);

	vector<int> boid_grid_coord = GetGridCoord(new_vector_index);
	boid.SetGridCoord(boid_grid_coord);
}

/**
 * \brief   Turns 3D cell co-ordinates into a 1D index so the grid can be represented
 *			by a 1D vector to guarantee contiguous memory and hence enable fast access.
 * \param  grid_index | (x,y,z) vector that gives cells 3D grid co-ordinates 
 * \return  | 1D grid vector index of the cell
 */
int SpatialGrid::GetGridVectorIndex(vector<int>& grid_index) const
{
	return cell_num * cell_num*grid_index[0] + cell_num * grid_index[1] + grid_index[2];
}

/**
 * \brief   Turns 3D cell co-ordinates into a 1D index so the grid can be represented
 *			by a 1D vector to guarantee contiguous memory and hence enable fast access.
 * \param  x | Cell x co-ordinate
 * \param  y | Cell y co-ordinate 
 * \param  z | Cell z co-ordinate 
 * \return  | 1D grid vector index of the cell
 */
int SpatialGrid::GetGridVectorIndex(int & x, int & y, int & z) const
{
	return x * cell_num*cell_num + y * cell_num + z;
}

/**
 * \brief   Uses a boids position to work out which grid cell it currently is in.
 * \param  boid | Boid to work out co-ordinates
 * \return  | Grid co-ordinates of boid
 */
vector<int> SpatialGrid::GetGridCoord(Boid & boid)
{
	vector<int> grid_pos(3);
	
	for (int i = 0; i < 3; i++)
	{
		grid_pos[i] = floor(boid.GetPosition()[i] / cell_length);

		if (!(grid_pos[i] < cell_num))
		{
			grid_pos[i] = cell_num - 1; // fixes issue where if boid position is exactly length floor(pos/length) out of bounds of allowed grid indexes
		}

	}

	return grid_pos;

}

/**
 * \brief   Converts 1D grid vector index into corresponding 3D grid cell co-ordinates
 * \param  vector_index | 1D Grid vector index to convert
 * \return  | Grid cell co-ordinates
 */
vector<int> SpatialGrid::GetGridCoord(int & vector_index)
{
	vector<int> return_value(3);
	return_value[0] = vector_index / (cell_num*cell_num);
	return_value[1] = (vector_index - return_value[0] * cell_num*cell_num) / cell_num;
	return_value[2] = vector_index - return_value[0] * cell_num*cell_num - return_value[1] * cell_num;
	return return_value;
}

/**
 * \brief   Finds appropriate grid cell location of a boid and adds it to the grid.
 * \param  boid | Boid to add to the grid
 */
void SpatialGrid::AddBoid(Boid & boid)
{
	vector<int> grid_index = GetGridCoord(boid);
	int grid_vector_index = GetGridVectorIndex(grid_index);
	grid[grid_vector_index].push_back(&boid);
	boid.SetGridCoord(grid_index);
}
