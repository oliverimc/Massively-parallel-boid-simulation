#pragma once
#include "pch.h"
#include "boid.h"
#include "spatial_grid.h" 
#include "communication.h"
#include "Eigen/Dense"
#include <vector>
#include <math.h>
#include <cstdio>

vector<Vector3f> run_worker(int rank, int size);
