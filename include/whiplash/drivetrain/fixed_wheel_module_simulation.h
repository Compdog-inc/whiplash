#pragma once

#include <memory>
#include <vector>
#include <span>

#include "wheel_module_simulation.h"
#include "whiplash/motor/motorcontroller.h"

namespace whiplash
{
    namespace drivetrain
    {
        class FixedWheelModuleSimulation : public WheelModuleSimulation
        {
        public:
            FixedWheelModuleSimulation(
                WheelModuleSimulationConfig config,
                std::span<b2Transform> wheelTransforms,
                std::unique_ptr<motor::MotorController> motorController);

            FixedWheelModuleSimulation(const FixedWheelModuleSimulation &) = delete;
            FixedWheelModuleSimulation &operator=(const FixedWheelModuleSimulation &) = delete;

            FixedWheelModuleSimulation(FixedWheelModuleSimulation &&) = default;
            FixedWheelModuleSimulation &operator=(FixedWheelModuleSimulation &&) = default;

            std::vector<b2Vec2> updateAndGetWheelForces(std::span<b2Vec2> currentWheelGroundWorldVelocities, b2Rot robotRotation, std::span<float> wheelGravityForces) override;

        private:
            float getOutputTorque();

            std::unique_ptr<motor::MotorController> motorController;

            float motorAppliedVoltage = 0.0f;
            float motorStatorCurrent = 0.0f;
            float wheelPosition = 0.0f;
            float wheelSpeed = 0.0f;
        };

    } // namespace drivetrain
} // namespace whiplash