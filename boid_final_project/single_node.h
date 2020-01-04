#pragma once
#include "preprocessor.h"
#include "single_node.h"
#include "boid.h"
#include "spatial_grid.h"
#include <fstream>
#include "Eigen/Dense"
#include <mpi.h>
#include <random>
#include <vector>
#include "omp.h"




vector<Vector3f> run(int rank, int size);