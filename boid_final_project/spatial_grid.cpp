#include "pch.h"
#include "spatial_grid.h"
#include <iostream>

SpatialGrid::SpatialGrid(vector<Boid> &boids)
{
	cell_num = floor(LENGTH / SEEING_DISTANCE);
	cell_length = float(LENGTH) / float(cell_num);
	grid.resize(cell_num*cell_num*cell_num);

	for (Boid &boid : boids)
	{
		AddBoid(boid);
	}
}


void SpatialGrid::UpdateNearCells(Boid & boid)
{
	vector<int> boid_grid_index = boid.GetGridIndex();
	int i = 0; 


	for (int x = -1; x < 2; x++)
	{
		for (int y = -1; y < 2; y++)
		{
			for (int z = -1; z < 2; z++)
			{
				int row_x = boid_grid_index[0] + x;
				int row_y = boid_grid_index[1] + y;
				int row_z = boid_grid_index[2] + z;

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

bool SpatialGrid::UpdateGrid(Boid & boid, vector<int>& update_tracker)
{
	vector<int> old_grid_index = boid.GetGridIndex();
	vector<int> new_grid_index = GetGridIndex(boid);

	if (old_grid_index != new_grid_index)
	{
		int old_vector_index = GetGridVectorIndex(old_grid_index);
		int new_vector_index = GetGridVectorIndex(new_grid_index);

		grid[old_vector_index].remove(&boid);
		grid[new_vector_index].push_back(&boid);
		
		boid.SetGridIndex(new_grid_index);
		update_tracker.push_back(old_vector_index);
		update_tracker.push_back(new_vector_index);
		
		return true;
	}
	
	else
	{
		return false;
	}
}

void SpatialGrid::UpdateGrid(Boid & boid, int old_vector_index, int new_vector_index)
{
	grid[old_vector_index].remove(&boid);
	grid[new_vector_index].push_back(&boid);
	vector<int> grid_index_pos = GetGridIndex(new_vector_index);
	boid.SetGridIndex(grid_index_pos);
}

int SpatialGrid::GetGridVectorIndex(vector<int>& grid_index) const
{
	return cell_num * cell_num*grid_index[0] + cell_num * grid_index[1] + grid_index[2];
}

int SpatialGrid::GetGridVectorIndex(int & x, int & y, int & z) const
{
	return x * cell_num*cell_num + y * cell_num + z;
}

vector<int> SpatialGrid::GetGridIndex(Boid & boid)
{
	vector<int> grid_pos(3);
	
	for (int i = 0; i < 3; i++)
	{
		grid_pos[i] = floor(boid.GetPosition()[i] / cell_length);

		if (!(grid_pos[i] < cell_num))
		{
			grid_pos[i] = cell_num - 1;
		}

	}

	return grid_pos;

}

vector<int> SpatialGrid::GetGridIndex(int & vector_index)
{
	vector<int> return_value(3);
	return_value[0] = vector_index / (cell_num*cell_num);
	return_value[1] = (vector_index - return_value[0] * cell_num*cell_num) / cell_num;
	return_value[2] = vector_index - return_value[0] * cell_num*cell_num - return_value[1] * cell_num;
	return return_value;
}

void SpatialGrid::AddBoid(Boid & boid)
{
vector<int> grid_index = GetGridIndex(boid);
int grid_vector_index = GetGridVectorIndex(grid_index);
grid[grid_vector_index].push_back(&boid);
boid.SetGridIndex(grid_index);
}
