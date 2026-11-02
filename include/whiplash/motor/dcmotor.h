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

            DCMotor(float nominalVoltage, float stallTorque, float stallCurrent, float freeCurrent, float freeSpeed, int numMotors = 1);

            float getCurrent(float velocity, float voltage) const;
            float getCurrent(float torque) const;
            float getTorque(float current) const;

            float getVoltage(float torque, float velocity) const;
            float getVelocity(float torque, float voltage) const;

            DCMotor withReduction(float reduction) const;
        };

        namespace DCMotors
        {
            enum class V5SmartMotorGearSet
            {
                // Just the internal gear ratio, use when the cartridge gear ratio is included as part of your mechanism ratio
                None,
                // 36:1 (100 RPM)
                Red,
                // 18:1 (200 RPM)
                Green,
                // 6:1 (600 RPM)
                Blue,
            };

            DCMotor V5_Smart_Motor_11W(int numMotors = 1, V5SmartMotorGearSet gearSet = V5SmartMotorGearSet::None);
        };

    } // namespace motor
} // namespace whiplash