#pragma once

#include <array>
#include <box2d/box2d.h>

namespace whiplash
{
    namespace math
    {

        template <std::size_t N>
        constexpr std::array<b2Vec2, N> flipPositions(const std::array<b2Vec2, N> &positions)
        {
            std::array<b2Vec2, N> flippedPositions;

            for (std::size_t i = 0; i < N; ++i)
            {
                flippedPositions[i] = b2Vec2{-positions[i].x, -positions[i].y};
            }

            return flippedPositions;
        }

        constexpr float applyDeadband(float value, float deadband, float maxMagnitude)
        {
            if (std::abs(value) < deadband)
            {
                return 0.0f;
            }
            else if (value > 0.0f)
            {
                return (1.0f + deadband / (maxMagnitude - deadband)) * (value - deadband);
            }
            else
            {
                return (1.0f + deadband / (maxMagnitude - deadband)) * (value + deadband);
            }
        }

    } // namespace math
} // namespace whiplash