#pragma once
#include "boid.h"
#include "omp.h"
#include <mpi.h>
#include <vector>


void DeSerialiseBoids(vector<Boid> &boids, vector<float> &memory);

void DeSerialiseBoids(vector<Boid> &boids, vector<float> &memory, int start, int end);

void SerialiseBoids(vector<Boid> &boids, vector<float> &memory);

void SerialiseBoids(vector<Boid> &boids, vector<float> &memory, int start, int end);

void BroadcastSendBoids(vector<Boid>& boids, vector<float>& memory, int rank);

void BroadcastRecieveBoids(vector<Boid>& boids, vector<float>& memory, int rank);
