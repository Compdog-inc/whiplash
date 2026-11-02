#pragma once

#include "dcmotor.h"

namespace whiplash
{
    namespace motor
    {
        struct ModuleMotorConfig
        {
            DCMotor motor;
            float gearRatio;
            float friction;

            ModuleMotorConfig(const DCMotor &motor, float gearRatio, float frictionVoltage);

            float calculateMechanismVelocity(float current, float voltage) const;

            float calculateCurrent(float mechanismVelocity, float voltage) const;

            float calculateCurrent(float torque) const;

            float calculateTorque(float current) const;
        };

    } // namespace motor
} // namespace whiplash