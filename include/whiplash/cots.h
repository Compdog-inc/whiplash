#pragma once

namespace whiplash
{
    namespace cots
    {
        // Coefficients of friction on competition foam tiles of various cots wheels
        namespace cof
        {
            // Anisotropic coefficient of friction, where X is the direction of wheel travel and Y is perpendicular to that direction
            struct COFPair
            {
                float x;
                float y;

                constexpr COFPair(float inTravel, float perpendicular)
                    : x(inTravel), y(perpendicular)
                {
                }

                constexpr COFPair(float isotropic)
                    : x(isotropic), y(isotropic)
                {
                }
            };

            static constexpr COFPair OMNI_WHEEL_2_75_INCH(1.3f, 0.15f);
            static constexpr COFPair OMNI_WHEEL_3_25_INCH(0.95f, 0.175f);
            static constexpr COFPair OMNI_WHEEL_4_INCH(1.3f, 0.15f);

            static constexpr COFPair RUBBERIZED_TRACKS(1.05f);
            static constexpr COFPair MESH_BRAKEPAD(0.86f);

            static constexpr COFPair TRACTION_WHEEL_4_INCH(0.65f);

        } // namespace cof
    } // namespace cots
} // namespace whiplash