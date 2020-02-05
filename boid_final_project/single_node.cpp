#include "pch.h"
#include "single_node.h"

/*! \file single_node.cpp
	\brief Implementation of the simulation for running on a single node


*/


using namespace std;
using namespace Eigen;

/**
 * \brief   Main function for executing simulation on a single node
 * \return  | Boid positions for each step of the simulation
 */
vector<Vector3f> run_single()
{
	int size = 1;
	random_device rand_dev;
	default_random_engine ran_num_gen(rand_dev());
	uniform_real_distribution<float> position_distribution(LENGTH / 4, 3 * LENGTH / 4);
	uniform_real_distribution<float> velocity_distribution(-MAX_SPEED, MAX_SPEED);

	vector<Boid> boids(BOID_NUMBER);
	vector<int> grid_updates;
	vector<Vector3f> paths(BOID_NUMBER*STEPS);


	for (Boid &boid : boids)
	{
		boid.SetRanValues(ran_num_gen, velocity_distribution, position_distribution);
	}


	SpatialGrid grid(boids);

	double start_time = MPI_Wtime();
	for (int step = 0; step < STEPS; step++)
	{
		grid_updates.resize(0);
		
		

		#pragma omp parallel for schedule(SCHEDULE)
		for (int boid = 0; boid < BOID_NUMBER; boid++)
		{


			grid.UpdateNearCells(boids[boid]);
			boids[boid].Update();
			paths[PathIndice(boid, step, BOID_NUMBER)] = boids[boid].GetPosition();

		}
		
		for (int boid = 0; boid < BOID_NUMBER; boid++)
		{
			grid.UpdateGrid(boids[boid], grid_updates, size);
		}
		

	}
	double end_time = MPI_Wtime();


	printf(" --------------------------------\n");
	printf("|  Number of Boids   |%10d|\n", BOID_NUMBER);
	printf(" --------------------------------\n");
	printf("|  Number of Steps   |%10d|\n", STEPS);
	printf(" -------------------------------\n");
	printf("|   Number of Nodes  |%10d|\n", size);
	printf(" -------------------------------\n");
	printf("|Number of Processors|%10d|\n", THREAD_NUM);
	printf(" --------------------------------\n");
	printf("|  Total Processors  |%10d|\n", size*THREAD_NUM);
	printf(" --------------------------------\n");
	printf("|    Time taken/s    |%10f|\n", end_time - start_time);
	printf(" --------------------------------\n");

	return paths;


}
