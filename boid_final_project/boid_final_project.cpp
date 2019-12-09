// boid_final_project.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "preprocessor.h"
#include "boid.h"
#include "spatial_grid.h"

#include <mpi.h>

#include <vector>
#include <random>
#include <iostream>
#include <fstream>

using namespace std;



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


	if (num_nodes == 1)
	{

		random_device rand_dev;
		default_random_engine ran_num_gen(rand_dev());
		uniform_real_distribution<float> posistion_distribution(LENGTH / 4, 3 * LENGTH / 4);
		uniform_real_distribution<float> velocity_distribution(-MAX_SPEED, -MAX_SPEED);
		
		vector<Boid> boids(BOID_NUMBER);
		vector<int> grid_updates;
		vector<Vector3f> paths(BOID_NUMBER*STEPS);

		for (Boid &boid : boids)
		{
			boid.SetRanValues(ran_num_gen, velocity_distribution, posistion_distribution);
		}


		SpatialGrid grid(boids);

		float start_t = MPI_Wtime();
		for (int step = 0; step < STEPS; step++)
		{
			grid_updates.resize(0);
			for (int boid = 0; boid < BOID_NUMBER; boid++)
			{
				
				
				grid.UpdateNearCells(boids[boid]);
				boids[boid].Update();
				grid.UpdateGrid(boids[boid], grid_updates);
				paths[PathIndice(boid,step,BOID_NUMBER)]= boids[boid].GetPosistion();

			}

		}
		float end_t = MPI_Wtime();

		printf("Time taken %.2f", end_t - start_t);
		write_to_file("first_test", paths, STEPS, BOID_NUMBER, rank);



	}

	else 

	{
		print("Not yet coded");
	}






	MPI_Finalize();
}