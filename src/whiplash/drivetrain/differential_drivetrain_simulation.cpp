#include "whiplash/drivetrain/differential_drivetrain_simulation.h"

namespace whiplash
{
    namespace drivetrain
    {

        DifferentialDrivetrainSimulation::DifferentialDrivetrainSimulation(b2WorldId world, DrivetrainSimulationConfig config, b2WorldTransform initialTransform)
            : DrivetrainSimulation(world, config, initialTransform)
        {
            b2Body_SetLinearDamping(bodyId, 1.4f);
            b2Body_SetAngularDamping(bodyId, 1.4f);
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
        }

    } // namespace drivetrain
} // namespace whiplash