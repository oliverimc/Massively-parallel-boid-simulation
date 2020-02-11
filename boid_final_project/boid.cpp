#include "pch.h"
#include "boid.h"

/*! \file boid.cpp
	\brief Implementation of the boid class
*/

/**
 * \brief  Constructor: Initializes internal variables and allocates required memory for cell and nearby boid buffers. 
 */
Boid::Boid()
{
	position_ = Vector3f::Zero();
	velocity_ = Vector3f::Zero();
	acceleration_ = Vector3f::Zero();
	grid_coord_.resize(SYS_DIM);
	neighbouring_cells_buffer_.resize(27);
	nearby_boid_buffer_.resize(BOID_NUMBER / BUFFER_FRACTION); // Over allocates to save time associated with dynamic allocation. 
}

/**
 * \brief  Main update loop. Finds nearby boids in local cells, calculates steering forces and weights them by provided coefficients.
 *		   Then updates kinematic variables. Boundary conditions are imposed and variables reset for next update loop.
 */
void Boid::Update()
{
    GetNearbyBoids();
	acceleration_ = COHESION_FACTOR * Cohesion(nearby_boid_buffer_) + SEPARATION_FACTOR * Separation(nearby_boid_buffer_) + ALIGNMENT_FACTOR * Alignment(nearby_boid_buffer_);
	velocity_ += acceleration_;
	position_ += velocity_;
	UpdateEdges();
	acceleration_ = Vector3f::Zero();
}

/**
 * \brief  Sets the boids position and velocity to a random value according to provided distributions. 
 * \param  random_engine | Random number generator
 * \param  vel_distr | Probability distribution of the velocity values
 * \param  pos_distr | Probability distribution of the position values
 */
void Boid::SetRanValues(default_random_engine & random_engine, uniform_real_distribution<float>& vel_distr, uniform_real_distribution<float>& pos_distr)
{
	for (int i = 0; i < SYS_DIM; i++)
	{
		velocity_[i] = vel_distr(random_engine);
		position_[i] = pos_distr(random_engine);
	}
}

/**
 * \brief  Serializes boid object into 6 floats in the provided vector at specified location.
 * \param  memory | Float vector where the values should be stored
 * \param  start_location | Index of the vector where the values should be stored from
 */
void Boid::Serialize(vector<float>& memory, int start_location)
{
	for (int i = 0; i < 3; i++)
	{
		memory[start_location + i] = position_[i];
		memory[start_location + 3 + i] = velocity_[i];
	}
}

/**
 * \brief  Deserializes boid object from  6 floats in vector.
 * \param  memory | Float vector to get values from
 * \param  start_location | Start index off the vector where values located
 */
void Boid::DeSerialize(vector<float>& memory, int start_location)
{
	for (int i = 0; i < 3; i++)
	{
		position_[i] = memory[start_location + i];
		velocity_[i] = memory[start_location + 3 + i];
	}
}

/**
  * \brief   Position vector getter
  * \return  | Position vector
  */
 Vector3f Boid::GetPosition() const
 {
	return position_;
}

/**
  * \brief   Velocity vector getter 
  * \return  | Velocity vector
  */
 Vector3f Boid::GetVelocity() const
 {
	return velocity_;
}

/**
 * \brief   Neighbouring cells getter
 * \return  | Neighbouring cells buffer
 */
vector<list<Boid*>*> Boid::GetNeighbourBuffer() const
{
	return neighbouring_cells_buffer_;
}

/**
 * \brief   Grid cell co-ordinates getter 
 * \return  | Grid cell co-ordinates
 */
vector<int> Boid::GetGridCoord() const
{
	return grid_coord_;
}

/**
 * \brief  Grid cell co-ordinates setter 
 * \param  grid_coord | Grid cell co-ordinates to set
 */
void Boid::SetGridCoord(vector<int>& grid_coord)
{
	grid_coord_ = grid_coord;
}

/**
 * \brief  Checks if boid position is out of bounds of simulation space and if so implements boundary conditions.
 */
void Boid::UpdateEdges()
{
	for (int i = 0; i < SYS_DIM; i++)
	{
		if (position_[i] > LENGTH)
		{
			position_[i] = 0;
		}
		else if (position_[i] < 0)
		{
			position_[i] = LENGTH;
		}   
	}
}

/**
 * \brief  Iterates over the cells provided by the grid and finds which boids are within range.
 *		   Then stores them in the buffer for use in steering calculations.
 */
void Boid::GetNearbyBoids()
{
	int i = 0;

	for (auto &cell : neighbouring_cells_buffer_)
	{
		for (auto& boid : *cell)
		{
		    float distance_squared = (boid->GetPosition() -position_).squaredNorm();
			
			if (distance_squared != 0 && distance_squared < SIGHT_RANGE_SQ)
			{
				//only calculates square root for boids that are nearby to reduce number of expensive calls to sqrt()
				
				get<0>(nearby_boid_buffer_[i]) = boid;
				get<1>(nearby_boid_buffer_[i]) = sqrt(distance_squared);
				i++;	
			}
		}
	}
		
	buffer_end_index_ = i; //So that the steering functions know where to iterate to
}



/**
 * \brief  Takes a vector and returns a vector in the same direction but a set magnitude
 * \param  vector | Vector to normalise
 * \param  magnitude | Magnitude to set the vector to 
 * \return  | Normalised Vector
 */
inline Vector3f Boid::NormaliseToMag(Vector3f & vector, float magnitude)
{
	return  vector.normalized()*magnitude;
}

/**
 * \brief  Calculates steering force due to Cohesion behaviour.
 *		   Boid tries to match it's velocity to average of neighbours.   
 * \param  nearby_boids | Nearby boid buffer to iterate over.
 * \return  | Acceleration due to cohesion steering behaviour
 */
Vector3f Boid::Cohesion(vector<tuple<Boid*, float>>& nearby_boids)
{
	int num_boids = 0;
	Vector3f average_vel = Vector3f::Zero();
	Vector3f correction_force = Vector3f::Zero();

	for (int index = 0 ; index < buffer_end_index_; index++)
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

/**
 * \brief   Calculates acceleration due to separation behaviour, boid tries to accelerate away from nearby boids.
 *			Effect weighted by how close neighbouring boid is by 1/r effect.
 * \param  nearby_boids | Nearby boid buffer to iterate over
 * \return  | Acceleration due to separation behaviour
 */
Vector3f Boid::Separation(vector<tuple<Boid*, float>>& nearby_boids)
{
	int num_boids = 0;
	Vector3f average_pos = Vector3f::Zero();
	Vector3f correction_force = Vector3f::Zero();

	for (int index = 0; index < buffer_end_index_; index++)
	{
		Vector3f pos_difference = position_ - get<0>(nearby_boid_buffer_[index])->GetPosition();
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

/**
 * \brief Calculates force due to alignment behaviour.
 *		  Boid tries to steer towards centre of mass of neighbours.
 * \param  nearby_boids | Nearby boid buffer to iterate over
 * \return  | Acceleration due to alignment behaviour
 */
Vector3f Boid::Alignment(vector<tuple<Boid*, float>>& nearby_boids)
{
	int num_boids = 0;
	Vector3f centre_mass = Vector3f::Zero();
	Vector3f correction_force = Vector3f::Zero();


	for (int index = 0; index < buffer_end_index_; index++)
	{
		centre_mass += get<0>(nearby_boid_buffer_[index])->GetPosition();
		num_boids++;
	}

	if (num_boids > 0)
	{
		centre_mass /= num_boids;
		Vector3f vector_to_com = centre_mass - position_;
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


