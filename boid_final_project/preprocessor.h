#pragma once

#define THREAD_NUM 4
#define MASTER 0 
#define SAVE true
#define SCHEDULE guided

#define LENGTH 1000
#define SEEING_DISTANCE 50
#define SEEING_DISTANCE_SQ SEEING_DISTANCE*SEEING_DISTANCE
#define BOID_NUMBER 2000
#define STEPS 1000

#define MAX_SPEED 3
#define MAX_FORCE 0.6

#define PI 3.14159265359


#define COHESION_FACTOR 1
#define ALIGNMENT_FACTOR 1
#define SEPARATION_FACTOR 1.05


#define print(val) std::cout<< val << std::endl;
#define PathIndice(boid,step,boid_number) step*boid_number+boid
#define MultiPathIndice(boid,step,boid_number,start) step*boid_number+(boid-start)