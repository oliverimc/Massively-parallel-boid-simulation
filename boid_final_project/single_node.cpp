#include "pch.h"
#include "single_node.h"

using namespace std;
using namespace Eigen;

vector<Vector3f> run(int rank, int size)
{

	random_device rand_dev;
	default_random_engine ran_num_gen(rand_dev());
	uniform_real_distribution<float> posistion_distribution(LENGTH / 4, 3 * LENGTH / 4);
	uniform_real_distribution<float> velocity_distribution(-MAX_SPEED, MAX_SPEED);

	vector<Boid> boids(BOID_NUMBER);
	vector<int> grid_updates;
	vector<Vector3f> paths(BOID_NUMBER*STEPS);


	for (Boid &boid : boids)
	{
		boid.SetRanValues(ran_num_gen, velocity_distribution, posistion_distribution);
	}


	SpatialGrid grid(boids);

	double start_t = MPI_Wtime();
	for (int step = 0; step < STEPS; step++)
	{
		grid_updates.resize(0);
		
		

		#pragma omp parallel for schedule(guided)
		for (int boid = 0; boid < BOID_NUMBER; boid++)
		{


			grid.UpdateNearCells(boids[boid]);
			boids[boid].Update();
			paths[PathIndice(boid, step, BOID_NUMBER)] = boids[boid].GetPosition();

		}
		
		for (int boid = 0; boid < BOID_NUMBER; boid++)
		{
			grid.UpdateGrid(boids[boid], grid_updates);
		}
		

	}
	double end_t = MPI_Wtime();


	printf("(%d)Time taken %.5f\n",rank, end_t - start_t);
	
	std::ofstream file;

	file.open("timing.txt", std::ofstream::app);
	file << THREAD_NUM * size << "," << end_t - start_t << std::endl;
	file.close();
	
	return paths;


}
