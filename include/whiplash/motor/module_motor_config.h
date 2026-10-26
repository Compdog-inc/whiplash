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

            ModuleMotorConfig(const DCMotor &motor, float gearRatio, float frictionVoltage)
                : motor(motor), gearRatio(gearRatio)
            {
                friction = motor.getTorque(motor.getCurrent(0.0f, frictionVoltage));
            }

            float calculateMechanismVelocity(float current, float voltage) const
            {
                return motor.getVelocity(motor.getTorque(current), voltage) / gearRatio;
            }

            float calculateCurrent(float mechanismVelocity, float voltage) const
            {
                return motor.getCurrent(mechanismVelocity * gearRatio, voltage);
            }

            float calculateCurrent(float torque) const
            {
                return motor.getCurrent(torque / gearRatio);
            }

            float calculateTorque(float current) const
            {
                return motor.getTorque(current) * gearRatio;
            }
        };

    } // namespace motor
} // namespace whiplash