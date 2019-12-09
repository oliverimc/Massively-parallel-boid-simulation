#include "pch.h"
#include "worker.h"


using namespace std;
using namespace Eigen;

vector<Vector3f> run_worker(int rank, int size)
{

	vector<Boid> boids(BOID_NUMBER);
	vector<float> boid_memory(BOID_NUMBER * 6);

	int boids_per_node = floor(BOID_NUMBER / size);
	int start_index = (rank - 1)*boids_per_node;
	int end_index = start_index + boids_per_node;

	BroadcastRecieveBoids(boids, boid_memory, MASTER);

	SpatialGrid grid(boids);

	return vector<Vector3f>(1);




}