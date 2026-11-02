#pragma once

#include "whiplash/motor/module_motor_config.h"
#include "whiplash/cots.h"

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

            // Anisotropic coefficient of friction
            cots::cof::COFPair coefficientOfFriction;

            // Number of wheels to take, in order, from the wheel transforms (mechanically linked)
            uint8_t numWheels;

            WheelModuleSimulationConfig(
                motor::DCMotor motor,
                float gearRatio,
                float frictionVoltage,
                float wheelRadiusMeters,
                cots::cof::COFPair coefficientOfFriction,
                uint8_t numWheels)
                : motorConfig(motor, gearRatio, frictionVoltage),
                  gearRatio(gearRatio),
                  frictionVoltage(frictionVoltage),
                  wheelRadiusMeters(wheelRadiusMeters),
                  coefficientOfFriction(coefficientOfFriction),
                  numWheels(numWheels)
            {
            }
        };

    } // namespace drivetrain
} // namespace whiplash