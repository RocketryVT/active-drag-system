#include <iostream>
#include <cstdlib>
#include "../include/surfaceFitModel.hpp"
#include "../include/actuationPlan.hpp"
#include "../include/ads.hpp"

int main() {
    
    SurfaceFitModel sfm = SurfaceFitModel();
    ActuationPlan plan = ActuationPlan(sfm);
    ADS ads = ADS(plan);
    ads.run();
    return EXIT_SUCCESS;
}
