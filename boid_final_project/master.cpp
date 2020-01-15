#include "pch.h"
#include "master.h"


using namespace std;
using namespace Eigen;

vector<Vector3f> run_master(int rank, int size)
{
	
	//print("MASTER");
	random_device rand_dev;
	default_random_engine ran_num_gen(rand_dev());
	uniform_real_distribution<float> position_distribution(LENGTH / 4, 3 * LENGTH / 4);
	uniform_real_distribution<float> velocity_distribution(-MAX_SPEED, MAX_SPEED);

	vector<Boid> boids(BOID_NUMBER);
	vector<float> boid_memory(BOID_NUMBER * 6); // pre-allocated contigous memory to de/serialise the boid data to for sending with MPI
	vector<int> grid_updates; //vector representing updates to the grid. Each update adds three integers: old spatial grid vector index, new grid vector index, boids vector index
	

	int boids_per_node = floor(BOID_NUMBER / size);
	int start_index = (size - 1)*boids_per_node;
	int end_index = boids.size();

	vector<Vector3f> paths(boids_per_node*STEPS);
	vector<float> node_boid_memory(boids_per_node * 6); // pre-allocated memory to de/sereialise boid data when sending to a from nodes.

	for (Boid &boid : boids)
	{
		boid.SetRanValues(ran_num_gen, velocity_distribution, position_distribution);
	}

	
	SpatialGrid grid(boids);

	BroadcastSendBoids(boids, boid_memory, MASTER);



	//printf("(%d) Starting master\n", rank);
	//fflush(stdout);

	double start_t = MPI_Wtime();
	for (int step = 0; step < STEPS; step++)
	{

		grid_updates.resize(0);
		

		#pragma omp parallel for
		for (int boid = start_index; boid < end_index; boid++)
		{

			grid.UpdateNearCells(boids[boid]);
			boids[boid].Update();			
			paths[MultiPathIndice(boid, step, boids_per_node, start_index)] = boids[boid].GetPosition();
			
		}
		


		for (int boid = start_index; boid < end_index; boid++)
		{
			if (grid.UpdateGrid(boids[boid], grid_updates))
			{
				grid_updates.push_back(boid);
			}
		}
		
		for (int node = 1; node < size; node++)
		{
			ReceiveBoids(boids, node_boid_memory, node, MASTER, (node - 1)*boids_per_node, node*boids_per_node);

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
	double end_t = MPI_Wtime();

	printf("%d:%d:%d:%d:%f\n", BOID_NUMBER, size, THREAD_NUM, size*THREAD_NUM, end_t - start_t);

	






	


	return paths;

	



}