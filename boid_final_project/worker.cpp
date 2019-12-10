#include "pch.h"
#include "worker.h"


using namespace std;
using namespace Eigen;

vector<Vector3f> run_worker(int rank, int size)
{

	vector<Boid> boids(BOID_NUMBER);
	vector<float> boid_memory(BOID_NUMBER * 6);
	vector<int> grid_updates;

	int boids_per_node = floor(BOID_NUMBER / size);
	int start_index = (rank - 1)*boids_per_node;
	int end_index = start_index + boids_per_node;

	vector<Vector3f> paths(boids_per_node*STEPS);
	vector<float> node_boid_memory(boids_per_node * 6);

	BroadcastRecieveBoids(boids, boid_memory, MASTER);

	SpatialGrid grid(boids);

	
	printf("(%d) Starting worker\n", rank);
	fflush(stdout);
	float start_t = MPI_Wtime();
	for (int step = 0; step < STEPS; step++)
	{
		grid_updates.resize(0);
		#pragma omp parallel for
		for (int boid = start_index; boid < end_index; boid++)
		{

			grid.UpdateNearCells(boids[boid]);
			boids[boid].Update();
			paths[MultiPathIndice(boid, step, boids_per_node, start_index)] = boids[boid].GetPosistion();

		}
	

		for (int boid = start_index; boid < end_index; boid++)
		{
			if (grid.UpdateGrid(boids[boid], grid_updates))
			{
				grid_updates.push_back(boid);
			}
		}
	

		SendBoids(boids, node_boid_memory, rank, MASTER, start_index, end_index);
		SendGridUpdates(grid_updates, MASTER);
		

		BroadcastReceiveGridUpdates(grid_updates, MASTER);
		BroadcastRecieveBoids(boids, boid_memory, MASTER);

	



		for (int i = 0; i < grid_updates.size(); i += 3)
		{
			
			grid.UpdateGrid(boids[grid_updates[i + 2]], grid_updates[i], grid_updates[i + 1]);

		}

	}
	float end_t = MPI_Wtime();

	printf("(%d)Total time taken %.2f\n", rank, end_t - start_t);




	return paths;




}