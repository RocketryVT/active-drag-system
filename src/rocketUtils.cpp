#include "../include/rocketUtils.hpp"

double deploy_percentage_to_angle(double percentage) {

    return (MAX_ANGLE - MIN_ANGLE) / 100.0 * percentage + MIN_ANGLE;
}


std::string format_data(std::string prefix, double data, int precision) {

    std::stringstream stream;
    stream << std::fixed << std::setprecision(precision) << data;
    std::string s = stream.str();
    return prefix + s;
}

bool led_out(Vehicle *vehicle) {

    std::ofstream file;
    file.open(LED_FILENAME);
    if (!file.is_open()) {
        return false;
    }

    file << std::to_string(vehicle->led_brightness);
    file.close();

    vehicle->led_time = time(nullptr);
    vehicle->led_brightness = (vehicle->led_brightness + 1) % 2;

    return true;
}

std::string state_for_log[5] = {"ON_PAD", "BOOST", "GLIDE", "APOGEE", "DONE"};
	