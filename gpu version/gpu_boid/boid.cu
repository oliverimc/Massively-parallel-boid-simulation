
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <vector>
#include <random>
#include <iostream>
#include <chrono>
#include <fstream>


#define BOID_NUMBER 1000
#define SAVE false
#define SIGHT_RANGE 100
#define SIGHT_RANGE_SQ SIGHT_RANGE*SIGHT_RANGE
#define LENGTH 1000
#define MAX_SPEED 3.0
#define MAX_FORCE 0.6
#define STEPS 1000
#define PathIndice(boid,step) step*BOID_NUMBER+boid

using namespace std;

/**
 * \brief  Returns the square of the Euclidean distance between two vectors
 * \param  x1 | vector 1 x component
 * \param  x2 | vector 2 x component
 * \param  y1 | vector 1 y component
 * \param  y2 | vector 2 y component
 * \param  z1 | vector 1 z component
 * \param  z2 | vector 2 z component
 * \return  | euclidean distance squared between the two vectors
 */
__device__
float SquareDistance(float &x1, float &x2, float &y1, float &y2, float &z1, float &z2)
{

	return pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2);
}

/**
 * \brief Returns the magnitude of a vector
 * \param  x | vector x component
 * \param  y | vector y component
 * \param  z | vector z component
 * \return  | magnitude of the vector
 */
__device__
float Magnitude(float &x, float &y, float &z)
{
	return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

/**
 * \brief  Returns the magnitude squared of a vector
 * \param  x | vector x component
 * \param  y | vector y component
 * \param  z | vector z component
 * \return  | magnitude squared
 */
__device__
float MagnitudeSq(float &x, float &y, float &z)
{
	return pow(x, 2) + pow(y, 2) + pow(z, 2);
}

/**
 * \brief  Updates the boids position according to the boid model.
 *		   One function which integrates cohesion,seperation and alignment
 * \param  vel_array | The array of floats that store the boids velocities
 * \param  pos_array | The array of floats that store the boids positions
 * \param  i | The index of the array where the given boids velocity/position can be found
 */
__device__
void UpdateBoid(float *vel_array, float *pos_array, int i)
{
	float pos_x = pos_array[i];
	float pos_y = pos_array[i + 1];
	float pos_z = pos_array[i + 2];

	float vel_x = vel_array[i];
	float vel_y = vel_array[i + 1];
	float vel_z = vel_array[i + 2];

	//Cohesion average velocity vector
	float cohesion_ave_vel_x = 0;
	float cohesion_ave_vel_y = 0;
	float cohesion_ave_vel_z = 0;

	//Seperation average position vector
	float seperation_ave_pos_x = 0;
	float seperation_ave_pos_y = 0;
	float seperation_ave_pos_z = 0;

	//Alignment average centre of mass vector
	float align_ave_com_x = 0;
	float align_ave_com_y = 0;
	float align_ave_com_z = 0;

	int num_boids = 0;

	for (int j = 0; j < BOID_NUMBER; j++)
	{
		float boid_pos_x = pos_array[3 * j];
		float boid_pos_y = pos_array[3 * j + 1];
		float boid_pos_z = pos_array[3 * j + 2];


		float sq_dist = SquareDistance(pos_x, boid_pos_x, pos_y, boid_pos_y, pos_z, boid_pos_z);

		if (sq_dist < SIGHT_RANGE_SQ && sq_dist != 0)
		{
			float boid_vel_x = vel_array[3 * j];
			float boid_vel_y = vel_array[3 * j + 1];
			float boid_vel_z = vel_array[3 * j + 2];

			num_boids++;

			cohesion_ave_vel_x += boid_vel_x;
			cohesion_ave_vel_y += boid_vel_y;
			cohesion_ave_vel_z += boid_vel_z;
			
			float dist = sqrt(sq_dist);
			seperation_ave_pos_x += (pos_x - boid_pos_x) / dist;
			seperation_ave_pos_y += (pos_y - boid_pos_y) / dist;
			seperation_ave_pos_z += (pos_z - boid_pos_z) / dist;
			
			align_ave_com_x += boid_pos_x;
			align_ave_com_y += boid_pos_y;
			align_ave_com_z += boid_pos_z;
		}
	}

	if (num_boids > 0)
	{
		align_ave_com_x /= num_boids;
		align_ave_com_y /= num_boids;
		align_ave_com_z /= num_boids;

		seperation_ave_pos_x /= num_boids;
		seperation_ave_pos_y /= num_boids;
		seperation_ave_pos_z /= num_boids;

		cohesion_ave_vel_x /= num_boids;
		cohesion_ave_vel_y /= num_boids;
		cohesion_ave_vel_z /= num_boids;

		//COHESION
		float cohesion_vec_magnitude = Magnitude(cohesion_ave_vel_x, cohesion_ave_vel_y, cohesion_ave_vel_z);
		cohesion_ave_vel_x = cohesion_ave_vel_x * MAX_SPEED / cohesion_vec_magnitude;
		cohesion_ave_vel_y = cohesion_ave_vel_y * MAX_SPEED / cohesion_vec_magnitude;
		cohesion_ave_vel_z = cohesion_ave_vel_z * MAX_SPEED / cohesion_vec_magnitude;

		float cohesion_correction_x = cohesion_ave_vel_x - vel_x;
		float cohesion_correction_y = cohesion_ave_vel_y - vel_y;
		float cohesion_correction_z = cohesion_ave_vel_z - vel_z;

		float cohesion_correction_magnitude = Magnitude(cohesion_correction_x, cohesion_correction_y, cohesion_correction_z);
		cohesion_correction_x = cohesion_correction_x * MAX_FORCE / cohesion_correction_magnitude;
		cohesion_correction_y = cohesion_correction_y * MAX_FORCE / cohesion_correction_magnitude;
		cohesion_correction_z = cohesion_correction_z * MAX_FORCE / cohesion_correction_magnitude;

		//SEPERATION
		float seperation_vec_magnitude_sq = MagnitudeSq(seperation_ave_pos_x, seperation_ave_pos_y, seperation_ave_pos_z);

		if (seperation_vec_magnitude_sq > 0) 
		{
			float seperation_vec_magnitude = sqrt(seperation_vec_magnitude_sq);
			seperation_ave_pos_x = seperation_ave_pos_x * MAX_SPEED / seperation_vec_magnitude;
			seperation_ave_pos_y = seperation_ave_pos_y * MAX_SPEED / seperation_vec_magnitude;
			seperation_ave_pos_z = seperation_ave_pos_z * MAX_SPEED / seperation_vec_magnitude;
		}

		float seperation_correction_x = seperation_ave_pos_x - vel_x;
		float seperation_correction_y = seperation_ave_pos_y - vel_y;
		float seperation_correction_z = seperation_ave_pos_z - vel_z;

		float seperation_correc_mag_sq = MagnitudeSq(seperation_correction_x, seperation_correction_y, seperation_correction_z);

		if (seperation_correc_mag_sq > MAX_FORCE*MAX_FORCE)
		{
			float seperation_correc_mag = sqrt(seperation_correc_mag_sq);
			seperation_correction_x = seperation_correction_x * MAX_FORCE / seperation_correc_mag;
			seperation_correction_y = seperation_correction_y * MAX_FORCE / seperation_correc_mag;
			seperation_correction_z = seperation_correction_z * MAX_FORCE / seperation_correc_mag;
		}

		//Alignment
		float vec_to_com_x = align_ave_com_x - pos_x;
		float vec_to_com_y = align_ave_com_y - pos_y;
		float vec_to_com_z = align_ave_com_z - pos_z;

		float com_vec_mag_sq = MagnitudeSq(vec_to_com_x, vec_to_com_y, vec_to_com_z);

		if (com_vec_mag_sq > 0)
		{
			float com_vec_mag = sqrt(com_vec_mag_sq);
			vec_to_com_x = vec_to_com_x * MAX_SPEED / com_vec_mag;
			vec_to_com_y = vec_to_com_y * MAX_SPEED / com_vec_mag;
			vec_to_com_z = vec_to_com_z * MAX_SPEED / com_vec_mag;
		}

		float align_correction_x = vec_to_com_x - vel_x;
		float align_correction_y = vec_to_com_y - vel_y;
		float align_correction_z = vec_to_com_z - vel_z;

		float align_correction_mag_sq = MagnitudeSq(align_correction_x, align_correction_y, align_correction_z);

		if (align_correction_mag_sq > MAX_FORCE*MAX_FORCE)
		{
			float align_correction_mag = sqrt(align_correction_mag_sq);
			align_correction_x = align_correction_x * MAX_FORCE / align_correction_mag;
			align_correction_y = align_correction_y * MAX_FORCE / align_correction_mag;
			align_correction_z = align_correction_z * MAX_FORCE / align_correction_mag;
		}

		vel_array[i] += align_correction_x + 1.05*seperation_correction_x + cohesion_correction_x;
		vel_array[i + 1] += align_correction_y + 1.05*seperation_correction_y + cohesion_correction_y;
		vel_array[i + 2] += align_correction_z + 1.05*seperation_correction_z + cohesion_correction_z;
	}

	//Update boids kinematics and impose boundary conditions
	pos_array[i] += vel_array[i];
	pos_array[i + 1] += vel_array[i + 1];
	pos_array[i + 2] += vel_array[i + 2];

	pos_array[i] = pos_array[i] > LENGTH ? 0 : pos_array[i];
	pos_array[i + 1] = pos_array[i + 1] > LENGTH ? 0 : pos_array[i + 1];
	pos_array[i + 2] = pos_array[i + 2] > LENGTH ? 0 : pos_array[i + 2];

	pos_array[i] = pos_array[i] < 0 ? LENGTH : pos_array[i];
	pos_array[i + 1] = pos_array[i + 1] < 0 ? LENGTH : pos_array[i + 1];
	pos_array[i + 2] = pos_array[i + 2] < 0 ? LENGTH : pos_array[i + 2];
}

/**
 * \brief  Main cuda kernel thar runs on the GPU updating all the boids,
 *		   distributing them to many threads.
 * \param  vel_array | Array of floats storing velocities of boids
 * \param  pos_array | Array of floats storing positions of boids
 */
__global__
void UpdateBoids(float *vel_array, float *pos_array)
{
	int index = blockIdx.x * blockDim.x + threadIdx.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = index; i < BOID_NUMBER; i += stride)
	{
		UpdateBoid(vel_array, pos_array, 3 * i);
	}
}

//Convenience structure for handling position vectors
struct vec3
{
	float x;
	float y;
	float z;
};

/**
 * \brief  Writes simulation data to a file
 * \param  name | Name of the file
 * \param  paths | Data of the simulation (boid trajectories)
 */
void WriteToFile(std::string name, std::vector<vec3> &paths)
{
	ofstream file;
	file.open(name + ".txt");

	file << "Boid Simulation Output Results:" << endl;
	file << "Time of Simulation: " << "None" << endl << endl;
	file << "Number of Boids: " << BOID_NUMBER << endl;
	file << "Size of Simulation Area: " << LENGTH << endl;
	file << "Number of Simulation Steps: " << STEPS << endl;
	file << endl;

	for (int step = 0; step < STEPS; step++)
	{
		for (int boid = 0; boid < BOID_NUMBER; boid++)
		{
			vec3 pos = paths[PathIndice(boid, step)];

			file << pos.x << ':' << pos.y << ":" << pos.z << "$";
		}
		file << "\n";
	}

	file.close();
}

int main()
{
	float* pos_array;
	float* vel_array;
	std::vector<vec3> paths(BOID_NUMBER*STEPS);

	//Allocate unified memory acessible by both host (PC) and device (GPU)
	cudaMallocManaged(&pos_array, 3 * BOID_NUMBER * sizeof(float));
	cudaMallocManaged(&vel_array, 3 * BOID_NUMBER * sizeof(float));

	random_device rand_dev;
	default_random_engine ran_num_gen(rand_dev());
	uniform_real_distribution<float> position_distribution(LENGTH / 4, 3 * LENGTH / 4);
	uniform_real_distribution<float> velocity_distribution(-MAX_SPEED, MAX_SPEED);

	//Assigns more threads as the boid number grows
	int blockSize = 256;
	int numBlocks = (BOID_NUMBER + blockSize - 1) / blockSize;

	for (int i = 0; i < 3 * BOID_NUMBER; i++)
	{
		pos_array[i] = position_distribution(ran_num_gen);
		vel_array[i] = velocity_distribution(ran_num_gen);
	}

	auto start = std::chrono::high_resolution_clock::now();
	for (int step = 0; step < STEPS; step++)
	{
		UpdateBoids << <numBlocks, blockSize >> > (vel_array, pos_array);
		cudaDeviceSynchronize();

		for (int boid = 0; boid < BOID_NUMBER; boid++)
		{
			paths[PathIndice(boid, step)].x = pos_array[boid * 3];
			paths[PathIndice(boid, step)].y = pos_array[boid * 3 + 1];
			paths[PathIndice(boid, step)].z = pos_array[boid * 3 + 2];
		}
	}
	
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

	std::cout << "Time Taken " << ": " << duration.count() / 1000.0 << "s" << endl;
	if (SAVE)
	{
		WriteToFile("gpu_test", paths);
	}
	cudaFree(vel_array);
	cudaFree(pos_array);

	return 0;
}




