#include "whiplash/drivetrain/differential_drivetrain_simulation.h"

namespace whiplash
{
    namespace drivetrain
    {

        DifferentialDrivetrainSimulation::DifferentialDrivetrainSimulation(b2WorldId world, DrivetrainSimulationConfig config, b2WorldTransform initialTransform)
            : DrivetrainSimulation(world, config, initialTransform)
        {
        }

        void DifferentialDrivetrainSimulation::simulationSubTick(int subTickNum)
        {
        }

    } // namespace drivetrain
} // namespace whiplash