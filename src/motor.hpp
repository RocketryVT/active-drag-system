#pragma once
#include "actuator.hpp"
#include "logger.hpp"

class Motor : public Actuator {

    private:


    public:

        Motor();

        virtual bool init() override;

        virtual bool writeData(void* data) override;
};