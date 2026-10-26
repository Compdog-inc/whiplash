#pragma once

#include "drivetrain_simulation.h"

namespace whiplash
{
    namespace drivetrain
    {

        class DifferentialDrivetrainSimulation : public DrivetrainSimulation
        {
        public:
            DifferentialDrivetrainSimulation(b2WorldId world, DrivetrainSimulationConfig config, b2WorldTransform initialTransform);

            DifferentialDrivetrainSimulation(const DifferentialDrivetrainSimulation &) = delete;
            DifferentialDrivetrainSimulation &operator=(const DifferentialDrivetrainSimulation &) = delete;

            void simulationSubTick(int subTickNum) override;
        };

    } // namespace drivetrain
} // namespace whiplash