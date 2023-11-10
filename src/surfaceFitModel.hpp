#pragma once
#include "eigen3/Eigen/Dense"
#include <cmath>

using namespace Eigen;

#define X_DEGREE 4 // Highest x-degree of current Surface Model
#define Y_DEGREE 3 // Highest y-degree of current Surface Model

class SurfaceFitModel {

    private:
        MatrixXd p; // Polynomial values

    public:
        SurfaceFitModel();

        double getFit(double x, double y);
};
