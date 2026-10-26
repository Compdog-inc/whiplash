#include "whiplash/drivetrain/wheel_module_simulation.h"

namespace whiplash
{
    namespace drivetrain
    {

        WheelModuleSimulation::WheelModuleSimulation(WheelModuleSimulationConfig config, std::span<b2Transform> wheelTransforms)
            : config(config), wheelTransforms(std::vector<b2Transform>(wheelTransforms.begin(), wheelTransforms.end()))
        {
        }

    } // namespace drivetrain
} // namespace whiplash