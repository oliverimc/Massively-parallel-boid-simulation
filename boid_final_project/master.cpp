#include "pch.h"
#include "master.h"


/*! \file master.cpp
	\brief Implementation of function for running simulation on master node


*/

using namespace std;
using namespace Eigen;


/**
 * \brief   Main function for executing simulation on a master node
 * \param  rank | MPI node rank
 * \param  size | Number of MPI nodes
 * \return  | Boid positions at each step for the masters portion of the simulation
 */
vector<Vector3f> run_master(int rank, int size)
{


	random_device rand_dev;
	default_random_engine ran_num_gen(rand_dev());
	uniform_real_distribution<float> position_distribution(LENGTH / 4, 3 * LENGTH / 4);
	uniform_real_distribution<float> velocity_distribution(-MAX_SPEED, MAX_SPEED);

	vector<Boid> boids(BOID_NUMBER);
	vector<float> boid_memory(BOID_NUMBER * 6); // pre-allocated contigous memory to de/serialise the boid data to for sending with MPI
	vector<int> grid_updates; //vector representing updates to the grid. Each update adds three integers: old spatial grid vector index, new grid vector index, boids vector index


	int boids_per_worker_node = floor(BOID_NUMBER / size);
	int boids_on_master = BOID_NUMBER / size + BOID_NUMBER % size;
	int start_index = (size - 1)*boids_per_worker_node;
	int end_index = boids.size();

	vector<Vector3f> paths(boids_on_master*STEPS);
	vector<float> node_boid_memory(boids_per_worker_node * 6); // pre-allocated memory to de/sereialise boid data when sending to a from nodes.

	for (Boid &boid : boids)
	{
		boid.SetRanValues(ran_num_gen, velocity_distribution, position_distribution);
	}


	SpatialGrid grid(boids);

	BroadcastSendBoids(boids, boid_memory, MASTER);



	double start_time = MPI_Wtime();
	for (int step = 0; step < STEPS; step++)
	{

		grid_updates.resize(0);


#pragma omp parallel for schedule(SCHEDULE)
		for (int boid = start_index; boid < end_index; boid++)
		{

			grid.UpdateNearCells(boids[boid]);
			boids[boid].Update();
			paths[MultiPathIndice(boid, step, boids_on_master, start_index)] = boids[boid].GetPosition();

		}



		for (int boid = start_index; boid < end_index; boid++)
		{
			if (grid.UpdateGrid(boids[boid], grid_updates, size))
			{
				grid_updates.push_back(boid);
			}
		}

		for (int node = 1; node < size; node++)
		{
			ReceiveBoids(boids, node_boid_memory, node, MASTER, (node - 1)*boids_per_worker_node, node*boids_per_worker_node);

		}

		for (int node = 1; node < size; node++)
		{
			vector<int> node_grid_updates;
			ReceiveGridUpdates(node_grid_updates, node);
			grid_updates.insert(grid_updates.end(), node_grid_updates.begin(), node_grid_updates.end());

		}

		for (unsigned int i = 0; i < grid_updates.size(); i += 3)
		{
			grid.UpdateGrid(boids[grid_updates[i + 2]], grid_updates[i], grid_updates[i + 1]);

		}


		BroadcastSendGridUpdates(grid_updates, MASTER);
		BroadcastSendBoids(boids, boid_memory, MASTER);









	}
	double end_time = MPI_Wtime();

	printf("*******Simulation Completed******\n");
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