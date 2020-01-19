#pragma once

constexpr auto THREAD_NUM = 4;
constexpr auto MASTER = 0 ;
constexpr auto SAVE = false;

constexpr auto BUFFER_FRACTION = 2 ;

constexpr auto LENGTH = 1000;
constexpr auto SEEING_DISTANCE = 50;
constexpr auto SEEING_DISTANCE_SQ = SEEING_DISTANCE * SEEING_DISTANCE;
constexpr auto BOID_NUMBER = 1000;
constexpr auto STEPS = 1000;

constexpr auto MAX_SPEED = 3.0;
constexpr auto MAX_FORCE = 0.6;

constexpr auto COHESION_FACTOR = 1;
constexpr auto ALIGNMENT_FACTOR = 1;
constexpr auto SEPARATION_FACTOR = 1.05;

#define SCHEDULE guided
#define print(val) std::cout<< val << std::endl;
#define PathIndice(boid,step,boid_number) step*boid_number+boid
#define MultiPathIndice(boid,step,boid_number,start) step*boid_number+(boid-start)