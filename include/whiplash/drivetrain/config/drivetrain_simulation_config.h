#pragma once

#include <box2d/box2d.h>
#include <vector>
#include "wheel_module_simulation_config.h"

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
        };

    } // namespace drivetrain
} // namespace whiplash