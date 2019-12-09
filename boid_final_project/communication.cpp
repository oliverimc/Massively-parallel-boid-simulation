#include "pch.h"
#include "communication.h"

void DeSerialiseBoids(vector<Boid>& boids, vector<float>& memory)
{
	for (int boid = 0; boid < boids.size(); boid++)
	{
		boids[boid].DeSerialise(memory, boid*6);
	}
}

void DeSerialiseBoids(vector<Boid>& boids, vector<float>& memory, int start, int end)
{
	for (int boid = start; boid < end; boid++)
	{
		boids[boid].DeSerialise(memory, (boid-start) * 6);
	}
}


void SerialiseBoids(vector<Boid>& boids, vector<float>& memory)
{
	for (int boid = 0; boid < boids.size(); boid++)
	{
		boids[boid].Serialise(memory, boid * 6);
	}
}

void SerialiseBoids(vector<Boid>& boids, vector<float>& memory, int start, int end)
{
	for (int boid = start; boid < end; boid++)
	{
		boids[boid].Serialise(memory, (boid - start) * 6);
	}
}

void BroadcastSendBoids(vector<Boid>& boids, vector<float>& memory, int rank)
{
	SerialiseBoids(boids, memory);
	MPI_Bcast(&memory[0], memory.size(), MPI_FLOAT, rank, MPI_COMM_WORLD);
}


void BroadcastRecieveBoids(vector<Boid>& boids, vector<float>& memory, int rank)
{

	MPI_Bcast(&memory[0], memory.size(), MPI_FLOAT, rank, MPI_COMM_WORLD);
	DeSerialiseBoids(boids, memory);

}

