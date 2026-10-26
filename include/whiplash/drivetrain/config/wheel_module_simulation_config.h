#pragma once

namespace whiplash
{
    namespace drivetrain
    {
        struct WheelModuleSimulationConfig
        {
            float gearRatio;
            float coefficientOfFriction;
            float frictionVoltage;
            float wheelRadiusMeters;

            // For omni wheels set this to a low value to allow the wheel to slide sideways.
            // For traction wheels set this to 1.0 to use the same cof in all directions.
            float perpendicularFrictionMultiplier;

            // Number of wheels to take, in order, from the wheel transforms (mechanically linked)
            uint8_t numWheels;
        };

    } // namespace drivetrain
} // namespace whiplash