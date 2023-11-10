#include "rocketUtils.hpp"

double deploy_percentage_to_angle(double percentage) {

    return (MAX_ANGLE - MIN_ANGLE) / 100.0 * percentage + MIN_ANGLE;
}


std::string format_data(std::string prefix, double data, int precision) {

    std::stringstream stream;
    stream << std::fixed << std::setprecision(precision) << data;
    std::string s = stream.str();
    return prefix + s;
}

std::string state_for_log[5] = {"ON_PAD", "BOOST", "GLIDE", "APOGEE", "DONE"};
	