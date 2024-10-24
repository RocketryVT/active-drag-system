#include "encoder.hpp"

encoder::encoder(i2c_inst_t* inst) {
	this->inst = inst;
}

void encoder::initialize() {
	//TODO: Figure out startup configuration necessary
}

bool encoder::validate() {
	//TODO: Figure out if any validation aside from data collection can actually be done with this chip
}
