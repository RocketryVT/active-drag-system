#include <cstdlib>
#include "surfaceFitModel.hpp"
#include "actuationPlan.hpp"
#include "ads.hpp"



int main(int argc, char *argv[]) {

    SurfaceFitModel sfm = SurfaceFitModel();
    ActuationPlan plan = ActuationPlan(sfm);
    ADS ads = ADS(plan);
    ads.run();

    return EXIT_SUCCESS;
}