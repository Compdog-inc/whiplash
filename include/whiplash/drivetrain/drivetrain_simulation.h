#pragma once

#include <box2d/box2d.h>
#include "whiplash/drivetrain/config/drivetrain_simulation_config.h"

namespace whiplash
{
    namespace drivetrain
    {

        class DrivetrainSimulation
        {
        public:
            static constexpr float FRAME_COEFFICIENT_OF_FRICTION = 0.65f;   // https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction
            static constexpr float FRAME_COEFFICIENT_OF_RESTITUTION = 0.08; // https://simple.wikipedia.org/wiki/Coefficient_of_restitution

            virtual ~DrivetrainSimulation();

            DrivetrainSimulation(const DrivetrainSimulation &) = delete;
            DrivetrainSimulation &operator=(const DrivetrainSimulation &) = delete;

            b2WorldTransform getTransform();

            b2Vec2 getLinearVelocity();
            float getAngularVelocity();

            virtual void simulationSubTick(int subTickNum) = 0;

        protected:
            DrivetrainSimulation(b2WorldId world, DrivetrainSimulationConfig config, b2WorldTransform initialTransform);

            DrivetrainSimulationConfig config;
            b2BodyId bodyId;
            b2ShapeId frameShapeId;
        };

    } // namespace drivetrain
} // namespace whiplash