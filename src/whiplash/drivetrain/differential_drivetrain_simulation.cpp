#include "whiplash/drivetrain/differential_drivetrain_simulation.h"

namespace whiplash
{
    namespace drivetrain
    {

        DifferentialDrivetrainSimulation::DifferentialDrivetrainSimulation(b2WorldId world, DrivetrainSimulationConfig &&config, b2WorldTransform initialTransform)
            : DrivetrainSimulation(world, std::move(config), initialTransform)
        {
            b2Body_SetLinearDamping(bodyId, 1.4f);
            b2Body_SetAngularDamping(bodyId, 1.4f);

            std::span<b2Transform> wheelTransformsSpan(this->config.wheelTransforms);
            size_t wheelTransformIndex = 0;
            for (size_t i = 0; i < this->config.moduleConfigs.size(); ++i)
            {
                wheelModules.emplace_back(
                    this->config.moduleConfigs[i],
                    wheelTransformsSpan.subspan(
                        wheelTransformIndex,
                        this->config.moduleConfigs[i].numWheels),
                    std::move(this->config.motorControllers[i]));
                wheelTransformIndex += this->config.moduleConfigs[i].numWheels;
            }
        }

        void DifferentialDrivetrainSimulation::simulationSubTick(int subTickNum)
        {
            applyFrictionForce();
            applyFrictionTorque();
            applyModuleForces();
        }

        void DifferentialDrivetrainSimulation::applyFrictionForce()
        {
        }

        void DifferentialDrivetrainSimulation::applyFrictionTorque()
        {
        }

        void DifferentialDrivetrainSimulation::applyModuleForces()
        {
            size_t wheelTransformIndex = 0;
            for (auto &module : wheelModules)
            {
                std::vector<b2Vec2> wheelVelocitiesWorld;
                wheelVelocitiesWorld.reserve(module.getConfig().numWheels);

                std::vector<float> wheelGravityForces;
                wheelGravityForces.reserve(module.getConfig().numWheels);

                for (size_t i = 0; i < module.getConfig().numWheels; ++i)
                {
                    wheelVelocitiesWorld.push_back(
                        b2Body_GetLocalPointVelocity(
                            bodyId,
                            config.wheelTransforms[wheelTransformIndex + i].p));
                    wheelGravityForces.push_back(
                        config.robotMassKg * 9.81f / config.wheelTransforms.size());
                }

                std::vector<b2Vec2> wheelForces = module.updateAndGetWheelForces(
                    std::span<b2Vec2>(wheelVelocitiesWorld),
                    b2Body_GetTransform(bodyId).q,
                    std::span<float>(wheelGravityForces));

                for (size_t i = 0; i < wheelForces.size(); ++i)
                {
                    b2Body_ApplyForce(
                        bodyId,
                        wheelForces[i],
                        b2Body_GetWorldPoint(
                            bodyId,
                            config.wheelTransforms[wheelTransformIndex + i].p),
                        true);
                }

                wheelTransformIndex += module.getConfig().numWheels;
            }
        }

    } // namespace drivetrain
} // namespace whiplash