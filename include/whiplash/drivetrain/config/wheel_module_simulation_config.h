#pragma once

#include "whiplash/motor/module_motor_config.h"

namespace whiplash
{
    namespace drivetrain
    {
        struct WheelModuleSimulationConfig
        {
            motor::ModuleMotorConfig motorConfig;

            float gearRatio;
            float frictionVoltage;
            float wheelRadiusMeters;

            // Anisotropic coefficient of friction, where X is the direction of wheel travel and Y is perpendicular to that direction
            float coefficientOfFrictionX;
            float coefficientOfFrictionY;

            // Number of wheels to take, in order, from the wheel transforms (mechanically linked)
            uint8_t numWheels;

            WheelModuleSimulationConfig(
                motor::DCMotor motor,
                float gearRatio,
                float frictionVoltage,
                float wheelRadiusMeters,
                float coefficientOfFrictionX,
                float coefficientOfFrictionY,
                uint8_t numWheels)
                : motorConfig(motor, gearRatio, frictionVoltage),
                  gearRatio(gearRatio),
                  frictionVoltage(frictionVoltage),
                  wheelRadiusMeters(wheelRadiusMeters),
                  coefficientOfFrictionX(coefficientOfFrictionX),
                  coefficientOfFrictionY(coefficientOfFrictionY),
                  numWheels(numWheels)
            {
            }
        };

    } // namespace drivetrain
} // namespace whiplash