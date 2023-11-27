#pragma once
#include <cmath>
#include "eigen3/Eigen/Dense"


using namespace Eigen;

#define X_DEGREE 4 // Highest x-degree of current Surface Fit Model
#define Y_DEGREE 3 // Highest y-degree of current Surface Fit Model

class SurfaceFitModel {

    private:
        MatrixXd p; // Polynomial values

    public:

        SurfaceFitModel();

        double getFit(double x, double y);
};
