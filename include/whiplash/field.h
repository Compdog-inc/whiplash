#pragma once

#include <box2d/box2d.h>
#include <vector>
#include <memory>
#include "drivetrain/drivetrain_simulation.h"

namespace whiplash
{

    class Field
    {
    public:
        virtual ~Field();

        Field(const Field &) = delete;
        Field &operator=(const Field &) = delete;

        Field(Field &&) = default;
        Field &operator=(Field &&) = default;

        static constexpr int SIMULATION_SUB_TICKS_IN_1_PERIOD = 5;

        static int getSimulationSubTicksIn1Period()
        {
            return SIMULATION_SUB_TICKS_IN_1_PERIOD;
        }

        static constexpr float SIMULATION_DT = 0.01f / SIMULATION_SUB_TICKS_IN_1_PERIOD;

        static float getSimulationDt()
        {
            return SIMULATION_DT;
        }

        void addDrivetrainSimulation(std::shared_ptr<drivetrain::DrivetrainSimulation> drivetrainSimulation);

        void simulationPeriodic();

        b2WorldId getWorldId()
        {
            return worldId;
        }

    protected:
        Field();

        virtual void simulationSubTick(int subTickNum);

        b2WorldId worldId;
        b2BodyId obstaclesId;
        std::vector<std::shared_ptr<drivetrain::DrivetrainSimulation>> drivetrainSimulations;
    };

} // namespace whiplash