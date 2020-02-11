#include "pch.h"
#include "preprocessor.h"
#include "single_node.h"
#include "master.h"
#include "worker.h"


#include "Eigen/Dense"
#include <mpi.h>
#include <vector>
#include <fstream>
#include <chrono>
#include <ctime>


/*! \file boid_final_project.cpp
	\brief Main execution file with entry point
*/

using namespace std;
using namespace Eigen;

/**
 * \brief Saves the boid simulation data (positions for each step) to a file.
 * \param name | What to name the file
 * \param paths | Vector of positions of each boid for every time step
 * \param steps | How many steps of data there are in the vector
 * \param boid_number | How many boids there are in the vector
 */
void WriteToFile(string name, vector<Vector3f> &paths, int steps, int boid_number)
{

	ofstream file;
	file.open(name + ".txt");
	
	auto now = chrono::system_clock::now();
	auto now_time = std::chrono::system_clock::to_time_t(now);

	file << "Boid Simulation Output Results:" << endl;
	file << "Time of Simulation: " << ctime(&now_time) << endl;
	file << "Number of Boids: " << BOID_NUMBER << endl;
	file << "Size of Simulation Area: " << LENGTH << endl;
	file << "Number of Simulation Steps: " << STEPS << endl;
	file << endl;

	for (int step = 0; step < steps; step++)
	{
		for (int boid = 0; boid < boid_number; boid++)
		{
			Vector3f position = paths[PathIndice(boid,step,boid_number)];

			for (int i = 0; i < SYS_DIM; i++)
			{
				file << position[i];
				if (i == (SYS_DIM-1))
				{
					file << "$";
				}
				else
				{
					file << ":";
				}
			}
		}

		file << endl;

	}

	file.close();
	
}


int main(int argc, char* argv[])
{
	int num_nodes, rank;
	MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &num_nodes);
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	
	omp_set_num_threads(THREAD_NUM);	
	
	if (num_nodes == 1)
	{
		vector<Vector3f> paths = run_single();
		if (SAVE) 
		{
			WriteToFile("single-node-results", paths, STEPS, BOID_NUMBER);
		}
	}

	else if(rank == MASTER)
	{
		vector<Vector3f> paths = run_master(rank, num_nodes);
		if (SAVE)
		{
			WriteToFile("multi-node-0", paths, STEPS, BOID_NUMBER/num_nodes+BOID_NUMBER%num_nodes);
		}
	}

	else
	{
		vector<Vector3f> paths = run_worker(rank, num_nodes);
		if (SAVE)
		{
			WriteToFile("multi-node-"+to_string(rank), paths, STEPS, BOID_NUMBER / num_nodes);
		}
	}
	   	  
	MPI_Finalize();
}