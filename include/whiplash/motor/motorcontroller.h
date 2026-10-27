#pragma once

namespace whiplash
{
    namespace motor
    {

        class MotorController
        {
        public:
            /**
             * Updates the control signal for the motor based on the current state of the mechanism and encoder.
             * @param mechanismAngle The current angle of the mechanism in radians.
             * @param mechanismVelocity The current velocity of the mechanism in radians per second.
             * @param encoderAngle The current angle of the encoder in radians.
             * @param encoderVelocity The current velocity of the encoder in radians per second.
             * @return The applied motor voltage in volts
             */
            virtual float updateControlSignal(
                float mechanismAngle,
                float mechanismVelocity,
                float encoderAngle,
                float encoderVelocity) = 0;

            virtual ~MotorController() = default;

            MotorController(const MotorController &) = delete;
            MotorController &operator=(const MotorController &) = delete;

            MotorController(MotorController &&) = default;
            MotorController &operator=(MotorController &&) = default;

        protected:
            MotorController() = default;
        };

    } // namespace motor
} // namespace whiplash
