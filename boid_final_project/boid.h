#pragma once
#include "pch.h"
#include "preprocessor.h"
#include "Eigen/Dense"
#include <vector>
#include <list>
#include <tuple>
#include <random>
#include <iostream>




using namespace Eigen;
using namespace std;

class Boid
{
public:
	Boid();
	~Boid() = default;

	void Update();
	void SetRanValues(default_random_engine &random_engine, uniform_real_distribution<float> &vel_distr, uniform_real_distribution<float> &pos_distr);
	
	void Serialise(vector<float> &memory, int start_location);
	void DeSerialise(vector<float> &memory, int start_location);


	Vector3f GetPosition() const;
    Vector3f GetVelocity() const;
	vector<list<Boid*>*> GetNeighbourBuffer() const;
	vector<int> GetGridIndex() const;
	void SetGridIndex(vector<int> &grid_index);
	vector<list<Boid*>*> neighbouring_cells_buffer_; //pre-allocated memory to store pointers to the 27 relevant cells containing boid pointers that are surrounding the boid in the spatial grid

private:
	Vector3f position_;
	Vector3f velocity_;
	Vector3f acceleration_;

	vector<int> grid_index_;

	vector<tuple<Boid*, float>> nearby_boid_buffer_; //pre-allocated memory for storing pointers to nearby boids and their distances which is then iterated through in Cohesion... etc
	int buffer_end_index_; // on each update stores how many boids were nearby and where to iterate to
	

	void UpdateEdges();
	void GetNearbyBoids();

	inline Vector3f NormaliseToMag(Vector3f &vector, float magnitude);

	Vector3f Cohesion(vector<tuple<Boid*, float>> &nearby_boids);
	Vector3f Separation(vector<tuple<Boid*, float>> &nearby_boids);
	Vector3f Alignment(vector<tuple<Boid*, float>> &nearby_boids);
	


};

