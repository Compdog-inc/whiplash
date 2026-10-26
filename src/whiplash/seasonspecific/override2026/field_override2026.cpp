#include "whiplash/seasonspecific/override2026/field_override2026.h"

#include <array>
#include "whiplash/mathutils.h"

namespace whiplash
{
    static constexpr float FIELD_X_MIN = -1.783207f;
    static constexpr float FIELD_X_MAX = 1.783207f;
    static constexpr float FIELD_Y_MIN = -1.783207f;
    static constexpr float FIELD_Y_MAX = 1.783207f;

    static constexpr b2Vec2 GOAL_CENTER_POSITION = {0.0f, 0.0f};

    static constexpr std::array<b2Vec2, 4>
        GOAL_RED_POSITIONS = {
            b2Vec2{-1.196117f, 0.598059f},
            b2Vec2{-1.196117f, -0.598059f},
            b2Vec2{-0.598059f, -1.196117f},
            b2Vec2{0.598059f, -1.196117f}};

    static constexpr std::array<b2Vec2, 4>
        GOAL_BLUE_POSITIONS =
            whiplash::math::flipPositions(GOAL_RED_POSITIONS);

    FieldOverride2026::FieldOverride2026()
        : Field()
    {
        b2Vec2 points[4] = {
            {FIELD_X_MIN, FIELD_Y_MIN},
            {FIELD_X_MIN, FIELD_Y_MAX},
            {FIELD_X_MAX, FIELD_Y_MAX},
            {FIELD_X_MAX, FIELD_Y_MIN},
        };

        b2ChainDef fieldPerimeterChainDef = b2DefaultChainDef();
        fieldPerimeterChainDef.points = points;
        fieldPerimeterChainDef.pointCount = 4;
        fieldPerimeterChainDef.isLoop = true;

        b2CreateChain(obstaclesId, &fieldPerimeterChainDef);

        createGoalShape(GOAL_CENTER_POSITION);
        for (const auto &goalPosition : GOAL_RED_POSITIONS)
        {
            createGoalShape(goalPosition);
        }
        for (const auto &goalPosition : GOAL_BLUE_POSITIONS)
        {
            createGoalShape(goalPosition);
        }
    }

    void FieldOverride2026::createGoalShape(b2Vec2 center)
    {
        std::vector<b2Vec2> vertices;

        auto addArc = [&](float startAngleDeg, float endAngleDeg, int segments, float radius)
        {
            float startRad = startAngleDeg * B2_PI / 180.0f;
            float endRad = endAngleDeg * B2_PI / 180.0f;
            for (int i = 0; i <= segments; ++i)
            {
                float t = (float)i / (float)segments;
                float angle = startRad + (endRad - startRad) * t;

                vertices.push_back(b2Vec2(
                    (radius * std::cosf(angle)) + center.x,
                    (radius * std::sinf(angle)) + center.y));
            }
        };

        addArc(29.87f, 60.13f, 6, 0.08187f);
        addArc(119.87f, 150.13f, 6, 0.08187f);
        addArc(209.87f, 240.13f, 6, 0.08187f);
        addArc(299.87f, 330.13f, 6, 0.08187f);

        b2ChainDef chainDef = b2DefaultChainDef();
        chainDef.points = vertices.data();
        chainDef.pointCount = vertices.size();
        chainDef.isLoop = true;

        b2CreateChain(obstaclesId, &chainDef);
    }
} // namespace whiplash