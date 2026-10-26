#pragma once

#include "wheel_module_simulation.h"

namespace whiplash
{
    namespace drivetrain
    {
        class FixedWheelModuleSimulation : public WheelModuleSimulation
        {
        public:
            FixedWheelModuleSimulation(WheelModuleSimulationConfig config, std::span<b2Transform> wheelTransforms);

            FixedWheelModuleSimulation(const FixedWheelModuleSimulation &) = delete;
            FixedWheelModuleSimulation &operator=(const FixedWheelModuleSimulation &) = delete;

            FixedWheelModuleSimulation(FixedWheelModuleSimulation &&) = default;
            FixedWheelModuleSimulation &operator=(FixedWheelModuleSimulation &&) = default;

            std::vector<b2Vec2> updateAndGetWheelForces(std::span<b2Vec2> currentWheelGroundWorldVelocities, b2Rot robotRotation, std::span<float> wheelGravityForces) override;

        private:
            float getOutputTorque();

            float motorAppliedVoltage = 0.0f;
            float motorStatorCurrent = 0.0f;
            float wheelPosition = 0.0f;
            float wheelSpeed = 0.0f;
        };

    } // namespace drivetrain
} // namespace whiplash