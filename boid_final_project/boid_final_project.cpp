// boid_final_project.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "preprocessor.h"
#include "single_node.h"
#include "master.h"
#include "worker.h"

#include<vector>
#include<Eigen/Dense>
#include <mpi.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;


void write_to_file(string name, vector<Vector3f> &paths, int steps, int boid_number, int rank)
{

	ofstream file;



	file.open(name + ".txt");


	file << "Boid Simulation Output Results:" << endl;
	file << "Time of Simulation: " << "None" << endl << endl;
	file << "Number of Boids: " << BOID_NUMBER << endl;
	file << "Size of Simulation Area: " << LENGTH << endl;
	file << "Number of Simulation Steps: " << STEPS << endl;
	file << endl;



	for (int step = 0; step < steps; step++)
	{
		for (int boid = 0; boid < boid_number; boid++)
		{
			Vector3f pos = paths[PathIndice(boid,step,boid_number)];
			for (int i = 0; i < 3; i++)
			{
				file << pos[i];
				if (i == (3 - 1))
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

	int num_nodes, rank, namelen;
	char processor_name[MPI_MAX_PROCESSOR_NAME];
	
	MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &num_nodes);
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Get_processor_name(processor_name, &namelen);

	print("Initialisation Done");

	omp_set_num_threads(THREAD_NUM);
	
	if (num_nodes == 1)
	{

		vector<Vector3f> paths = run(rank, num_nodes);
		if (SAVE) 
		{
			write_to_file("Test2", paths, STEPS, BOID_NUMBER, rank);
		}
		

	}

	else if(rank == MASTER)

	{
		vector<Vector3f> paths = run_master(rank, num_nodes);
		
		if (SAVE)
		{
			write_to_file("mulit-node-0", paths, STEPS, BOID_NUMBER/num_nodes+BOID_NUMBER%num_nodes, rank);
		}
	}

	else
	{
		vector<Vector3f> paths = run_worker(rank, num_nodes);

		if (SAVE)
		{
			write_to_file("multi-node-"+to_string(rank), paths, STEPS, BOID_NUMBER / num_nodes, rank);
		}

	}






	MPI_Finalize();
}