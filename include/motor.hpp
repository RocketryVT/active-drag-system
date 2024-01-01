#pragma once
#include "actuator.hpp"
#include "logger.hpp"
#include "rocketUtils.hpp"

class Motor : public Actuator {

    private:


    public:

        Motor();

        virtual bool init(void* data) override;

        virtual bool writeData(void* data) override;
};