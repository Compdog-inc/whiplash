#pragma once

#include <box2d/box2d.h>

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

            virtual void simulationSubTick(int subTickNum) = 0;

        protected:
            WheelModuleSimulation(b2WorldTransform initialTransform);

            b2BodyId bodyId;
        };

    } // namespace drivetrain
} // namespace whiplash