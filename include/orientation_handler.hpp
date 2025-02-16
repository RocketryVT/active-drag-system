#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "kalman_filter.hpp"

using namespace Eigen;

class orientation_handler {
	private:
		//Internal filter object, stores state and runs normally
		kalman_filter kf;

	public:
		//Default constructor
		orientation_handler();

		
};
