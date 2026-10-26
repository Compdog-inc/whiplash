#pragma once

#include <box2d/box2d.h>
#include <span>
#include <vector>
#include "whiplash/drivetrain/config/wheel_module_simulation_config.h"

namespace whiplash
{
    namespace drivetrain
    {
        class WheelModuleSimulation
        {
        public:
            virtual ~WheelModuleSimulation() = default;

            WheelModuleSimulation(const WheelModuleSimulation &) = delete;
            WheelModuleSimulation &operator=(const WheelModuleSimulation &) = delete;

            WheelModuleSimulation(WheelModuleSimulation &&) = default;
            WheelModuleSimulation &operator=(WheelModuleSimulation &&) = default;

            virtual std::vector<b2Vec2> updateAndGetWheelForces(std::span<b2Vec2> currentWheelGroundWorldVelocities, b2Rot robotRotation, std::span<float> wheelGravityForces) = 0;

            WheelModuleSimulationConfig getConfig() const
            {
                return config;
            }

        protected:
            WheelModuleSimulation(WheelModuleSimulationConfig config, std::span<b2Transform> wheelTransforms);

            WheelModuleSimulationConfig config;
            std::vector<b2Transform> wheelTransforms;
        };

    } // namespace drivetrain
} // namespace whiplash