#include "pch.h"
#include "master.h"


using namespace std;
using namespace Eigen;

vector<Vector3f> run_master(int rank, int size)
{
	random_device rand_dev;
	default_random_engine ran_num_gen(rand_dev());
	uniform_real_distribution<float> posistion_distribution(LENGTH / 4, 3 * LENGTH / 4);
	uniform_real_distribution<float> velocity_distribution(-MAX_SPEED, -MAX_SPEED);

	vector<Boid> boids(BOID_NUMBER);
	vector<float> boid_memory(BOID_NUMBER * 6);
	vector<int> local_grid_updates;


	int boids_per_node = floor(BOID_NUMBER / size);
	int start_index = (size - 1)*boids_per_node;
	int end_index = boids.size();


	for (Boid &boid : boids)
	{
		boid.SetRanValues(ran_num_gen, velocity_distribution, posistion_distribution);
		
	}

	SpatialGrid grid(boids);

	BroadcastSendBoids(boids, boid_memory, MASTER);
	

	return vector<Vector3f>(1);

	



}