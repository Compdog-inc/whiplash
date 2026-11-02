#include "whiplash/motor/module_motor_config.h"

namespace whiplash
{
    namespace motor
    {
        ModuleMotorConfig::ModuleMotorConfig(const DCMotor &motor, float gearRatio, float frictionVoltage)
            : motor(motor), gearRatio(gearRatio)
        {
            friction = motor.getTorque(motor.getCurrent(0.0f, frictionVoltage));
        }

        float ModuleMotorConfig::calculateMechanismVelocity(float current, float voltage) const
        {
            return motor.getVelocity(motor.getTorque(current), voltage) / gearRatio;
        }

        float ModuleMotorConfig::calculateCurrent(float mechanismVelocity, float voltage) const
        {
            return motor.getCurrent(mechanismVelocity * gearRatio, voltage);
        }

        float ModuleMotorConfig::calculateCurrent(float torque) const
        {
            return motor.getCurrent(torque / gearRatio);
        }

        float ModuleMotorConfig::calculateTorque(float current) const
        {
            return motor.getTorque(current) * gearRatio;
        }
    } // namespace motor
} // namespace whiplash