#include "whiplash/drivetrain/fixed_wheel_module_simulation.h"

#include "whiplash/field.h"
#include "whiplash/mathutils.h"

#include <box2d/math_functions.h>
#include <algorithm>

namespace whiplash
{
    namespace drivetrain
    {

        FixedWheelModuleSimulation::FixedWheelModuleSimulation(
            WheelModuleSimulationConfig config,
            std::span<b2Transform> wheelTransforms,
            std::unique_ptr<motor::MotorController> motorController)
            : WheelModuleSimulation(config, wheelTransforms),
              motorController(std::move(motorController))
        {
        }

        std::vector<b2Vec2> FixedWheelModuleSimulation::updateAndGetWheelForces(std::span<b2Vec2> currentWheelGroundWorldVelocities, b2Rot robotRotation, std::span<float> wheelGravityForces)
        {
            float outputTorque = getOutputTorque();

            float nominalTorquePerWheel = outputTorque / static_cast<float>(config.numWheels);
            float nominalForceX = nominalTorquePerWheel / config.wheelRadiusMeters;

            std::vector<b2Vec2> wheelForces;
            wheelForces.reserve(config.numWheels);

            float accumulatedWheelSpeedFromGround = 0.0f;
            bool anyWheelSkidding = false;

            for (int i = 0; i < config.numWheels; ++i)
            {
                b2Rot wheelRotation = b2MulRot(robotRotation, wheelTransforms[i].q);

                b2Vec2 localVelocity = b2InvRotateVector(wheelRotation, currentWheelGroundWorldVelocities[i]);

                float maxGripForceX = wheelGravityForces[i] * config.coefficientOfFrictionX;
                float maxGripForceY = wheelGravityForces[i] * config.coefficientOfFrictionY;

                float forceX = nominalForceX;
                bool isSkiddingX = std::abs(forceX) > maxGripForceX;
                if (isSkiddingX)
                {
                    forceX = std::copysign(maxGripForceX, forceX);
                    anyWheelSkidding = true;
                }

                float slipThreshold = 0.005f;
                float forceY;
                if (std::abs(localVelocity.y) > slipThreshold)
                {
                    forceY = std::copysign(maxGripForceY, -localVelocity.y);
                }
                else
                {
                    float lateralStiffness = 10000.0f;
                    forceY = std::clamp(-lateralStiffness * localVelocity.y, -maxGripForceY, maxGripForceY);
                }

                accumulatedWheelSpeedFromGround += localVelocity.x / config.wheelRadiusMeters;

                b2Vec2 localForceVector = {forceX, forceY};
                b2Vec2 worldForceVector = b2RotateVector(wheelRotation, localForceVector);

                wheelForces.push_back(worldForceVector);
            }

            wheelSpeed = accumulatedWheelSpeedFromGround / static_cast<float>(config.numWheels);

            if (anyWheelSkidding)
            {
                float equilibriumSpeed = config.motorConfig.calculateMechanismVelocity(
                    config.motorConfig.calculateCurrent(
                        nominalForceX * config.wheelRadiusMeters),
                    motorAppliedVoltage);
                wheelSpeed = wheelSpeed * 0.5f + equilibriumSpeed * 0.5f;
            }

            wheelPosition += wheelSpeed * Field::getSimulationDt();

            return wheelForces;
        }

        float FixedWheelModuleSimulation::getOutputTorque()
        {
            motorAppliedVoltage = motorController->updateControlSignal(
                wheelPosition,
                wheelSpeed,
                wheelPosition * config.gearRatio,
                wheelSpeed * config.gearRatio);

            motorStatorCurrent = config.motorConfig.calculateCurrent(wheelSpeed, motorAppliedVoltage);

            float outputTorque = config.motorConfig.calculateTorque(motorStatorCurrent);

            float outputTorqueWithFriction = math::applyDeadband(
                outputTorque,
                config.motorConfig.friction,
                std::numeric_limits<float>::infinity());

            return outputTorqueWithFriction;
        }

    } // namespace drivetrain
} // namespace whiplash