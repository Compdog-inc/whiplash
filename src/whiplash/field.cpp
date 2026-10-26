#include "whiplash/field.h"

namespace whiplash
{

    Field::~Field()
    {
        drivetrainSimulations.clear();

        if (B2_IS_NON_NULL(obstaclesId))
        {
            b2DestroyBody(obstaclesId);
        }

        if (B2_IS_NON_NULL(worldId))
        {
            b2DestroyWorld(worldId);
        }
    }

    Field::Field()
    {
        b2WorldDef worldDef = b2DefaultWorldDef();

        worldDef.workerCount = 4;
        worldDef.userTaskContext = this;
        worldDef.enableSleep = true;
        worldDef.gravity = b2Vec2{0.0f, 0.0f};

        worldId = b2CreateWorld(&worldDef);

        b2BodyDef bodyDef = b2DefaultBodyDef();
        bodyDef.type = b2_staticBody;
        obstaclesId = b2CreateBody(worldId, &bodyDef);
    }

    void Field::addDrivetrainSimulation(std::shared_ptr<drivetrain::DrivetrainSimulation> drivetrainSimulation)
    {
        drivetrainSimulations.emplace_back(drivetrainSimulation);
    }

    void Field::simulationPeriodic()
    {
        for (int subTickNum = 0; subTickNum < SIMULATION_SUB_TICKS_IN_1_PERIOD; ++subTickNum)
        {
            simulationSubTick(subTickNum);
        }
    }

    void Field::simulationSubTick(int subTickNum)
    {
        for (auto &drivetrainSimulation : drivetrainSimulations)
        {
            drivetrainSimulation->simulationSubTick(subTickNum);
        }

        b2World_Step(worldId, SIMULATION_DT, 1);
    }

} // namespace whiplash