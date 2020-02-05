#pragma once

/*! \file preprocessor.h
	\brief Constant definitions and Macro functions


*/

/**
 * \brief  Number of threads system uses. 
 */
constexpr auto THREAD_NUM = 1;
/**
 * \brief  Convince definition for referring to master rank
 */
constexpr auto MASTER = 0 ;
/**
 * \brief  Flag to set if paths data will be saved. 
 */
constexpr auto SAVE = true;
/**
 * \brief  Sets boids buffer size for nearby boids. Defined in terms of total number of boids.
 *		   i.e 4-> buffer size = boid_number/4
 *		   Buffer needs to be as big as largest possible number of neighbours a boid can have.
 *		   Hence value dependent on Seeing distance.
 *		   Experimentally seeing distance = 100 , fraction = 4 and 200 , 2 are appropriate.
 */
constexpr auto BUFFER_FRACTION = 4 ;
/**
 * \brief  Length of simulation area, arbitary units.
 *		   Visualization tool built around 1000.
 *		   Change can cause undefined visualization behaviour
 */
constexpr auto LENGTH = 1000;
/**
 * \brief  Cutoff range for which neighbours cause effect on a boid. 
 */
constexpr auto SEEING_DISTANCE = 100;
/**
 * \brief  Convenience definition for distance magnitude comparisons. 
 */
constexpr auto SEEING_DISTANCE_SQ = SEEING_DISTANCE * SEEING_DISTANCE;
/**
 * \brief  Number of boids in the simulation 
 */
constexpr auto BOID_NUMBER = 2000;
/**
 * \brief  Number of steps to run the simulation for 
 */
constexpr auto STEPS = 1000;
/**
 * \brief  Max speed to which velocity differentials are normalised to.
 *		   Used in calculation of steering forces.
 */
constexpr auto MAX_SPEED = 3.0;
/**
 * \brief  Max Force used to normalise final steering forces. In combination
 *		   with max speed can be altered to effect behaviour of the system.
 */
constexpr auto MAX_FORCE = 0.5;
/**
 * \brief  Weighting factor for cohesion acceleration component
 *		   Altering changes behaviour of the system
 */
constexpr auto COHESION_FACTOR = 1.01;
/**
 * \brief  Weighting factor for alignment acceleration component
 *		   Altering changes behaviour of the system
 */
constexpr auto ALIGNMENT_FACTOR = 1;
/**
 * \brief  Weighting factor for separation acceleration component
 *		   Altering changes behaviour of the system
 */
constexpr auto SEPARATION_FACTOR = 1.06;
/**
 * \brief  Type of OpenMP thread distribution to split work for thread team 
 */
#define SCHEDULE guided
/**
 * \brief   Convenience print function with new line
 * \param  val | Value to be printed.
 */
#define print(val) std::cout<< val << std::endl;
/**
 * \brief  Multi-dimensional indexing of 1D paths vector 
 * \param  boid | boid index
 * \param  step | step index
 * \param  boid_number |  number of boids stored in the vector
 */
#define PathIndice(boid,step,boid_number) step*boid_number+boid
/**
 * \brief  Way to index a 1D vector of paths for a selection of boids (ie a nodes share of the work)
 * \param  boid | boid index
 * \param  step | step index
 * \param  boid_number | number of boids stored in the vector
 * \param  start | boid index for the start of the section of the vector the node is responsible for.
 */
#define MultiPathIndice(boid,step,boid_number,start) step*boid_number+(boid-start)