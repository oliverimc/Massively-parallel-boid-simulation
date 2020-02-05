#pragma once
#include "boid.h"
#include "omp.h"
#include <mpi.h>
#include <vector>




void DeSerializeBoids(vector<Boid> &boids, vector<float> &memory);

void DeSerializeBoids(vector<Boid> &boids, vector<float> &memory, int start, int end);

void SerializeBoids(vector<Boid> &boids, vector<float> &memory);

void SerializeBoids(vector<Boid> &boids, vector<float> &memory, int start, int end);

void BroadcastSendBoids(vector<Boid>& boids, vector<float>& memory, int rank);

void BroadcastReceiveBoids(vector<Boid>& boids, vector<float>& memory, int rank);

void SendBoids(vector<Boid>& boids, vector<float>& memory, int destination, int start, int stop);

void ReceiveBoids(vector<Boid>& boids, vector<float>& memory, int source, int destination, int start, int stop);

void SendGridUpdates(vector<int>& updates, int destination);

void ReceiveGridUpdates(vector<int>& updates, int source);

void BroadcastSendGridUpdates(vector<int>& updates, int source);

void BroadcastReceiveGridUpdates(vector<int>& updates, int source);
