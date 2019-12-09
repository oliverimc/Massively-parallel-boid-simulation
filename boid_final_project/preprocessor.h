#pragma once

#define THREAD_NUM 4
#define MASTER 0 
#define SAVE false

#define LENGTH 1000
#define SEEING_DISTANCE 100
#define SEEING_DISTANCE_SQ SEEING_DISTANCE*SEEING_DISTANCE
#define BOID_NUMBER 10
#define STEPS 1000

#define MAX_SPEED 3
#define MAX_FORCE 0.6


#define COHESION_FACTOR 1
#define ALIGNMENT_FACTOR 1
#define SEPERATION_FACTOR 1.05

#define print(val) std::cout<< val << std::endl;
#define PathIndice(boid,step,boid_number) step*boid_number+boid