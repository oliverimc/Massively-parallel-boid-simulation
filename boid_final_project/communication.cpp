#include "pch.h"
#include "communication.h"

void DeSerialiseBoids(vector<Boid>& boids, vector<float>& memory)
{
	for (unsigned int boid = 0; boid < boids.size(); boid++)
	{
		boids[boid].DeSerialise(memory, boid*6);
	}
}

void DeSerialiseBoids(vector<Boid>& boids, vector<float>& memory, int start, int end)
{
	for ( int boid = start; boid < end; boid++)
	{
		boids[boid].DeSerialise(memory, (boid-start) * 6);
	}
}


void SerialiseBoids(vector<Boid>& boids, vector<float>& memory)
{
	for ( unsigned int boid = 0; boid < boids.size(); boid++)
	{
		boids[boid].Serialise(memory, boid * 6);
	}
}

void SerialiseBoids(vector<Boid>& boids, vector<float>& memory, int start, int end)
{
	for ( int boid = start; boid < end; boid++)
	{
		boids[boid].Serialise(memory, (boid - start) * 6);
	}
}

void BroadcastSendBoids(vector<Boid>& boids, vector<float>& memory, int rank)
{
	SerialiseBoids(boids, memory);
	MPI_Bcast(&memory[0], memory.size(), MPI_FLOAT, rank, MPI_COMM_WORLD);
}


void BroadcastReceiveBoids(vector<Boid>& boids, vector<float>& memory, int rank)
{

	MPI_Bcast(&memory[0], memory.size(), MPI_FLOAT, rank, MPI_COMM_WORLD);
	DeSerialiseBoids(boids, memory);

}

void SendBoids(vector<Boid>& boids, vector<float>& memory, int source, int destination, int start, int stop)
{
	SerialiseBoids(boids, memory, start, stop);
	MPI_Send(&memory[0], memory.size(), MPI_FLOAT, destination, 5, MPI_COMM_WORLD);



}

void ReceiveBoids(vector<Boid>& boids, vector<float>& memory, int source, int destination, int start, int stop)
{
	MPI_Status stat;
	MPI_Recv(&memory[0], memory.size(), MPI_FLOAT, source, 5, MPI_COMM_WORLD, &stat);
	DeSerialiseBoids(boids, memory, start, stop);

}


void SendGridUpdates(vector<int> &updates, int destination)
{
	int size = updates.size();
	MPI_Send(&size, 1, MPI_INT, destination, 6, MPI_COMM_WORLD);
	if (size > 0)
	{
		MPI_Send(&updates[0], size, MPI_INT, destination, 6, MPI_COMM_WORLD);
	}
	else 
	{
		updates.resize(0);
	}


}

void ReceiveGridUpdates(vector<int> &updates, int source)
{

	int size;
	MPI_Status stat;
	MPI_Recv(&size, 1, MPI_INT, source, 6, MPI_COMM_WORLD, &stat);
	if (size > 0)
	{
		updates.resize(size);
		MPI_Recv(&updates[0], size, MPI_INT, source, 6, MPI_COMM_WORLD, &stat);

	}
	else 
	{
		updates.resize(0);
	}


}

void BroadcastSendGridUpdates(vector<int> &updates, int source)
{
	int size = updates.size();
	MPI_Bcast(&size, 1, MPI_INT, source, MPI_COMM_WORLD);
	if (size > 0)
	{
		MPI_Bcast(&updates[0], size, MPI_INT, source, MPI_COMM_WORLD);
	}

}

void BroadcastReceiveGridUpdates(vector<int> &updates, int source)
{
	int size;
	MPI_Bcast(&size, 1, MPI_INT, source, MPI_COMM_WORLD);
	if (size > 0)
	{
		updates.resize(size);
		MPI_Bcast(&updates[0], size, MPI_INT, source, MPI_COMM_WORLD);
	}


}