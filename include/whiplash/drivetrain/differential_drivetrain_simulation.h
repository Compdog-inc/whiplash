#pragma once

#include "drivetrain_simulation.h"
#include "fixed_wheel_module_simulation.h"

namespace whiplash
{
    namespace drivetrain
    {

        class DifferentialDrivetrainSimulation : public DrivetrainSimulation
        {
        public:
            DifferentialDrivetrainSimulation(b2WorldId world, DrivetrainSimulationConfig &&config, b2WorldTransform initialTransform);

            DifferentialDrivetrainSimulation(const DifferentialDrivetrainSimulation &) = delete;
            DifferentialDrivetrainSimulation &operator=(const DifferentialDrivetrainSimulation &) = delete;

            DifferentialDrivetrainSimulation(DifferentialDrivetrainSimulation &&) = default;
            DifferentialDrivetrainSimulation &operator=(DifferentialDrivetrainSimulation &&) = default;

            void simulationSubTick(int subTickNum) override;

        private:
            void applyFrictionForce();
            void applyFrictionTorque();
            void applyModuleForces();

            std::vector<FixedWheelModuleSimulation> wheelModules;
        };

    } // namespace drivetrain
} // namespace whiplash