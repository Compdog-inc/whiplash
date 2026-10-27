#include "whiplash/drivetrain/drivetrain_simulation.h"

namespace whiplash
{
    namespace drivetrain
    {

        DrivetrainSimulation::DrivetrainSimulation(b2WorldId world, DrivetrainSimulationConfig &&config, b2WorldTransform initialTransform)
            : config(std::move(config))
        {
            b2BodyDef bodyDef = b2DefaultBodyDef();
            bodyDef.type = b2BodyType::b2_dynamicBody;
            bodyDef.position = initialTransform.p;
            bodyDef.rotation = initialTransform.q;
            bodyDef.linearDamping = 0.1f;
            bodyDef.angularDamping = 0.1f;

            bodyId = b2CreateBody(world, &bodyDef);

            b2ShapeDef shapeDef = b2DefaultShapeDef();
            shapeDef.density = config.robotMassKg / (config.frameLengthX * config.frameWidthY);
            shapeDef.material.friction = FRAME_COEFFICIENT_OF_FRICTION;
            shapeDef.material.restitution = FRAME_COEFFICIENT_OF_RESTITUTION;

            b2Polygon box = b2MakeBox(config.frameLengthX / 2.0f, config.frameWidthY / 2.0f);
            b2CreatePolygonShape(bodyId, &shapeDef, &box);
        }

        DrivetrainSimulation::~DrivetrainSimulation()
        {
            if (B2_IS_NON_NULL(bodyId))
            {
                b2DestroyBody(bodyId);
            }
        }

        b2WorldTransform DrivetrainSimulation::getTransform()
        {
            return b2Body_GetTransform(bodyId);
        }

        b2Vec2 DrivetrainSimulation::getLinearVelocity()
        {
            return b2Body_GetLinearVelocity(bodyId);
        }

        float DrivetrainSimulation::getAngularVelocity()
        {
            return b2Body_GetAngularVelocity(bodyId);
        }

        void DrivetrainSimulation::setTransform(b2Pos position, b2Rot rotation)
        {
            b2Body_SetTransform(bodyId, position, rotation);
        }

    } // namespace drivetrain
} // namespace whiplash