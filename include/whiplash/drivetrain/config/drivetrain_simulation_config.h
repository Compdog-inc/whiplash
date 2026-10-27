#pragma once

#include <memory>
#include <box2d/box2d.h>
#include <vector>
#include "wheel_module_simulation_config.h"

#include "whiplash/motor/motorcontroller.h"

namespace whiplash
{
    namespace drivetrain
    {
        struct DrivetrainSimulationConfig
        {
            float robotMassKg;
            float frameWidthY;
            float frameLengthX;

            std::vector<WheelModuleSimulationConfig> moduleConfigs;
            std::vector<b2Transform> wheelTransforms;
            std::vector<std::unique_ptr<motor::MotorController>> motorControllers;
        };

    } // namespace drivetrain
} // namespace whiplash