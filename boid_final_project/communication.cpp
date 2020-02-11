#include "pch.h"
#include "communication.h"

/*! \file communication.cpp
	\brief Functions to handle inter-node communication of data
*/

/**
 * \brief  Deserializes all boids represented in float memory to vector of boid objects.
 * \param  boids | Boid vector to deserialize to
 * \param  memory | Flot vector to deserialize from
 */
void DeSerializeBoids(vector<Boid>& boids, vector<float>& memory)
{
	for (int boid = 0; boid < boids.size(); boid++)
	{
		boids[boid].DeSerialize(memory, boid * SYS_DIM * 2);
	}
}

/**
 * \brief  Deserializes boids in a selected range of the float memory to vector of boid objects.
 * \param  boids | Boid vector to deserialize to
 * \param  memory | Float vector to deserialize from
 * \param  start | Vector start index
 * \param  end | Vector index
 */
void DeSerializeBoids(vector<Boid>& boids, vector<float>& memory, int start, int end)
{
	for (int boid = start; boid < end; boid++)
	{
		boids[boid].DeSerialize(memory, (boid - start) *SYS_DIM * 2);
	}
}

/**
 * \brief  Serializes all boids from vector of boid objects to vector of floats.
 * \param  boids | Boid vector to serialize from
 * \param  memory | Float vector to serialize to
 */
void SerializeBoids(vector<Boid>& boids, vector<float>& memory)
{
	for (int boid = 0; boid < boids.size(); boid++)
	{
		boids[boid].Serialize(memory, boid * *SYS_DIM * 2);
	}
}

/**
 * \brief  Serializes selected boids from vector of boid objects to vector of floats.
 * \param  boids | Boid vector to serialize from
 * \param  memory | Float vector to serialize to
 * \param  start | Boid vector index to start serializing at
 * \param  end | Boid vector indext to end serializing at
 */
void SerializeBoids(vector<Boid>& boids, vector<float>& memory, int start, int end)
{
	for (int boid = start; boid < end; boid++)
	{
		boids[boid].Serialize(memory, (boid - start) * SYS_DIM * 2);
	}
}

/**
 * \brief  MPI broadcast operation integrated with serialization routine allowing node to directly broadcast vector of boid objects.
 * \param  boids | Vector of boids to objects
 * \param  memory | Intermediary float vector to hold values for MPI broadcast routine
 * \param  rank | MPI Broadcast root rank
 */
void BroadcastSendBoids(vector<Boid>& boids, vector<float>& memory, int rank)
{
	SerializeBoids(boids, memory);
	MPI_Bcast(&memory[0], memory.size(), MPI_FLOAT, rank, MPI_COMM_WORLD);
}

/**
 * \brief  MPI broadcast operation integrated with deserialization routine
 *		   allowing to node to receive a broadcast of boids from other nodes
 * \param  boids | Boid vector to receive to
 * \param  memory | Intermediary float vector for MPI to broadcast to
 * \param  rank | MPI Broadcast rank to receive from
 */
void BroadcastReceiveBoids(vector<Boid>& boids, vector<float>& memory, int rank)
{
	MPI_Bcast(&memory[0], memory.size(), MPI_FLOAT, rank, MPI_COMM_WORLD);
	DeSerializeBoids(boids, memory);
}

/**
 * \brief  MPI Send routine integrated with serialization
 *		   allowing direct send of selection of vector of boid objects.
 * \param  boids | Boid vector to send
 * \param  memory | Intermediary float vector to serialize to, from which MPI can send from
 * \param  destination | Destination node MPI rank
 * \param  start | Boid vector start index of selection to send
 * \param  stop | Boid vector end index of selection to send
 */
void SendBoids(vector<Boid>& boids, vector<float>& memory, int destination, int start, int stop)
{
	SerializeBoids(boids, memory, start, stop);
	MPI_Send(&memory[0], memory.size(), MPI_FLOAT, destination, 5, MPI_COMM_WORLD);
}

/**
 * \brief  MPI receive routine integrated with deserialization
 *		   allowing direct receiving of vector of boids from another node
 * \param  boids | Vector of boids to receive to
 * \param  memory | Intermediary float vector for MPI to receive to and be deserialized
 * \param  source | Source node MPI rank
 * \param  destination | Destination node MPI rank
 * \param  start | Boid vector start index for where to deserialize the selection to
 * \param  stop | Boid vector end index for where to deserialize the selection to
 */
void ReceiveBoids(vector<Boid>& boids, vector<float>& memory, int source, int destination, int start, int stop)
{
	MPI_Status stat;
	MPI_Recv(&memory[0], memory.size(), MPI_FLOAT, source, 5, MPI_COMM_WORLD, &stat);
	DeSerializeBoids(boids, memory, start, stop);
}

/**
 * \brief  Double MPI Send routine enabling sending of dynamically sized vector containing grid updates.
 *		   Allows sending of grid update data to keep nodes copy of spatial grid in sync.
 * \param  updates | Grid updates vector containing data to send
 * \param  destination | Destination node MPI rank
 */
void SendGridUpdates(vector<int> &updates, int destination)
{
	int size = updates.size();
	MPI_Send(&size, 1, MPI_INT, destination, 6, MPI_COMM_WORLD);
	if (size > 0)
	{
		MPI_Send(&updates[0], size, MPI_INT, destination, 6, MPI_COMM_WORLD);
	}
}

/**
 * \brief  Double MPI receive routine allowing receiving of dynamic grid updates vector
 *		   to allow nodes spatial grid to stay in sync with others.
 * \param  updates | Grid updates vector to write data to.
 * \param  source | Source node MPI rank
 */
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

/**
 * \brief  Double broadcast MPI operation enables node to broadcast
 *		   dynamic sized grid updates vectors to other nodes.
 * \param  updates | Vector of grid updates
 * \param  source | Source node MPI rank
 */
void BroadcastSendGridUpdates(vector<int> &updates, int source)
{
	int size = updates.size();
	MPI_Bcast(&size, 1, MPI_INT, source, MPI_COMM_WORLD);
	if (size > 0)
	{
		MPI_Bcast(&updates[0], size, MPI_INT, source, MPI_COMM_WORLD);
	}
}

/**
 * \brief  Double broadcast receive operation to enable node to receive
 *		   dynamic sized grid updates vector from other nodes.
 * \param  updates | Vector of grid updates
 * \param  source | Source node MPI rank
 */
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