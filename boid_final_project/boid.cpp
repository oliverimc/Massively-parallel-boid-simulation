#include "pch.h"
#include "boid.h"
#include <iostream>


Boid::Boid()
{
	posistion_ = Vector3f::Zero();
	velocity_ = Vector3f::Zero();
	acceleration_ = Vector3f::Zero();
	grid_index_.resize(3);
	neighbouring_cells_buffer_.resize(27);
	nearby_boid_buffer_.resize(BOID_NUMBER / 4);
}

Boid::~Boid()
{
	
}

void Boid::Update()
{
    GetNearbyBoids();
	acceleration_ = COHESION_FACTOR * Cohesion(nearby_boid_buffer_) + SEPERATION_FACTOR * Seperation(nearby_boid_buffer_) + ALIGNMENT_FACTOR * Alignment(nearby_boid_buffer_);
	velocity_ += acceleration_;
	posistion_ += velocity_;
	UpdateEdges();
	acceleration_ = Vector3f::Zero();
}

void Boid::SetRanValues(default_random_engine & random_engine, uniform_real_distribution<float>& vel_distr, uniform_real_distribution<float>& pos_distr)
{
	for (int i = 0; i < 3; i++)
	{
		velocity_[i] = vel_distr(random_engine);
		posistion_[i] = pos_distr(random_engine);
	}
}

void Boid::Serialise(vector<float>& memory, int start_location)
{
	for (int i = 0; i < 3; i++)
	{
		memory[start_location + i] = posistion_[i];
		memory[start_location + 3 + i] = velocity_[i];

	}


}

void Boid::DeSerialise(vector<float>& memory, int start_location)
{
	for (int i = 0; i < 3; i++)
	{
		posistion_[i] = memory[start_location + i];
		velocity_[i] = memory[start_location + 3 + i];

	}

}

const Vector3f Boid::GetPosistion()
{
	return posistion_;
}

const Vector3f Boid::GetVelocity()
{
	return velocity_;
}

vector<list<Boid*>*> Boid::GetNeighbourBuffer()
{
	return neighbouring_cells_buffer_;
}

const vector<int> Boid::GetGridIndex()
{
	return grid_index_;
		
}

void Boid::SetGridIndex(vector<int>& grid_index)
{
	grid_index_ = grid_index;
}

void Boid::UpdateEdges()
{
	for (int i = 0; i < 3; i++)
	{
		if (posistion_[i] > LENGTH)
		{
			posistion_[i] = 0;
		}
		else if (posistion_[i] < 0)
		{
			posistion_[i] = LENGTH;
		}
			   
	}

}

void Boid::GetNearbyBoids()
{
	int i = 0;

	for (auto &cell : neighbouring_cells_buffer_)
	{
		
		
		for (auto boid = cell->begin(); boid != cell->end(); ++boid)
		{
			float distance_squared = ((*boid)->GetPosistion() -posistion_).squaredNorm();
			
			if (distance_squared != 0 && distance_squared < SEEING_DISTANCE_SQ)
			{
				//only calculates square root for boids that are nearby to reduce number of expensive calls to Sqrt()
				get<0>(nearby_boid_buffer_[i]) = *boid;
				get<1>(nearby_boid_buffer_[i]) = sqrt(distance_squared);
				i++;
					
			}
		}
	}

	
	buffer_end_index_ = i;
}

Vector3f Boid::NormaliseToMag(Vector3f & vector, float magnitude)
{
	return  vector.normalized()*magnitude;
}

Vector3f Boid::Cohesion(vector<tuple<Boid*, float>>& nearby_boids)
{
	int num_boids = 0;
	Vector3f average_vel = Vector3f::Zero();
	Vector3f correction_force = Vector3f::Zero();

	for (int index =0 ; index<buffer_end_index_; index++)
	{
		average_vel += get<0>(nearby_boid_buffer_[index])->GetVelocity();
		num_boids++;

	}

	if (num_boids > 0)
	{
		average_vel /= num_boids;
		average_vel = NormaliseToMag(average_vel, MAX_SPEED);
		correction_force = average_vel - velocity_;
		correction_force = NormaliseToMag(correction_force, MAX_FORCE);

	}

	return correction_force;
}

Vector3f Boid::Seperation(vector<tuple<Boid*, float>>& nearby_boids)
{
	int num_boids = 0;
	Vector3f average_pos = Vector3f::Zero();
	Vector3f correction_force = Vector3f::Zero();

	for (int index = 0; index < buffer_end_index_; index++)
	{
		Vector3f pos_difference = posistion_ - get<0>(nearby_boid_buffer_[index])->GetPosistion();
		pos_difference /= get<1>(nearby_boid_buffer_[index]);
		average_pos += pos_difference;
		num_boids++;
	}

	if (num_boids > 0)
	{
		average_pos /= num_boids;

		if (average_pos.squaredNorm() > 0)
		{
			average_pos = NormaliseToMag(average_pos, MAX_SPEED);
		}

		correction_force = average_pos - velocity_;

		if (correction_force.squaredNorm() > MAX_FORCE*MAX_FORCE)
		{
			correction_force = NormaliseToMag(correction_force, MAX_FORCE);
		}

	}
	return correction_force;

}

Vector3f Boid::Alignment(vector<tuple<Boid*, float>>& nearby_boids)
{
	int num_boids = 0;
	Vector3f centre_mass = Vector3f::Zero();
	Vector3f correction_force = Vector3f::Zero();


	for (int index = 0; index < buffer_end_index_; index++)
	{
		centre_mass += get<0>(nearby_boid_buffer_[index])->GetPosistion();
		num_boids++;
	}

	if (num_boids > 0)
	{
		centre_mass /= num_boids;
		Vector3f vector_to_com = centre_mass - posistion_;
		if (vector_to_com.squaredNorm() > 0)
		{
			vector_to_com = NormaliseToMag(vector_to_com, MAX_SPEED);
		}

		correction_force = vector_to_com - velocity_;

		if (correction_force.squaredNorm() > MAX_FORCE*MAX_FORCE)
		{
			correction_force = NormaliseToMag(correction_force, MAX_FORCE);
		}

	}

	return correction_force;




}


