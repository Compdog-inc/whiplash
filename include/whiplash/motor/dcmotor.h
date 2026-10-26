#pragma once

namespace whiplash
{
    namespace motor
    {
        struct DCMotor
        {
            float nominalVoltage;
            float stallTorque;
            float stallCurrent;
            float freeCurrent;
            float freeSpeed;
            float R;
            float Kv;
            float Kt;

            DCMotor(float nominalVoltage, float stallTorque, float stallCurrent, float freeCurrent, float freeSpeed)
                : nominalVoltage(nominalVoltage), stallTorque(stallTorque), stallCurrent(stallCurrent), freeCurrent(freeCurrent), freeSpeed(freeSpeed)
            {
                R = nominalVoltage / stallCurrent;
                Kv = freeSpeed / (nominalVoltage - R * freeCurrent);
                Kt = stallTorque / stallCurrent;
            }

            float getCurrent(float velocity, float voltage) const
            {
                return -1.0f / Kv / R * velocity + 1.0f / R * voltage;
            }

            float getCurrent(float torque) const
            {
                return torque / Kt;
            }

            float getTorque(float current) const
            {
                return current * Kt;
            }

            float getVoltage(float torque, float velocity) const
            {
                return 1.0f / Kv * velocity + 1.0f / Kt * R * torque;
            }

            float getVelocity(float torque, float voltage) const
            {
                return voltage * Kv - 1.0f / Kt * torque * R * Kv;
            }

            DCMotor withReduction(float reduction) const
            {
                return DCMotor(
                    nominalVoltage,
                    stallTorque * reduction,
                    stallCurrent,
                    freeCurrent,
                    freeSpeed / reduction);
            }
        };

    } // namespace motor
} // namespace whiplash