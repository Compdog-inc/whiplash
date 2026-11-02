#include "whiplash/motor/dcmotor.h"
#include <numbers>

namespace whiplash
{
    namespace motor
    {
        DCMotor::DCMotor(float nominalVoltage, float stallTorque, float stallCurrent, float freeCurrent, float freeSpeed, int numMotors)
            : nominalVoltage(nominalVoltage),
              stallTorque(stallTorque * numMotors),
              stallCurrent(stallCurrent * numMotors),
              freeCurrent(freeCurrent * numMotors),
              freeSpeed(freeSpeed)
        {
            R = nominalVoltage / this->stallCurrent;
            Kv = freeSpeed / (nominalVoltage - R * this->freeCurrent);
            Kt = this->stallTorque / (this->stallCurrent - this->freeCurrent);
        }

        float DCMotor::getCurrent(float velocity, float voltage) const
        {
            return -1.0f / Kv / R * velocity + 1.0f / R * voltage;
        }

        float DCMotor::getCurrent(float torque) const
        {
            return torque / Kt + freeCurrent;
        }

        float DCMotor::getTorque(float current) const
        {
            return (current - freeCurrent) * Kt;
        }

        float DCMotor::getVoltage(float torque, float velocity) const
        {
            return 1.0f / Kv * velocity + R * (torque / Kt + freeCurrent);
        }

        float DCMotor::getVelocity(float torque, float voltage) const
        {
            return voltage * Kv - R * Kv * (torque / Kt + freeCurrent);
        }

        DCMotor DCMotor::withReduction(float reduction) const
        {
            return DCMotor(
                nominalVoltage,
                stallTorque * reduction,
                stallCurrent,
                freeCurrent,
                freeSpeed / reduction);
        }

        namespace DCMotors
        {
            DCMotor V5_Smart_Motor_11W(int numMotors, V5SmartMotorGearSet gearSet)
            {
                float reduction;
                switch (gearSet)
                {
                case V5SmartMotorGearSet::Red:
                    reduction = 36.0f;
                    break;
                case V5SmartMotorGearSet::Green:
                    reduction = 18.0f;
                    break;
                case V5SmartMotorGearSet::Blue:
                    reduction = 6.0f;
                    break;
                default:
                    reduction = 1.0f;
                    break;
                }

                return DCMotor(
                           12.0f,                                       // voltage (V)
                           0.11083f,                                    // stall torque (Nm)
                           4.686f,                                      // stall current (A)
                           0.13982f,                                    // free current (A)
                           4352.724f / 60.0f * 2.0f * std::numbers::pi, // free speed (rpm to rad/s)
                           numMotors)
                    .withReduction(reduction);
            }
        } // namespace DCMotors
    } // namespace motor
} // namespace whiplash