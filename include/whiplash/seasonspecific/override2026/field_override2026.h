#pragma once

#include "whiplash/field.h"

namespace whiplash
{
    class FieldOverride2026 : public Field
    {
    public:
        FieldOverride2026();

        FieldOverride2026(const FieldOverride2026 &) = delete;
        FieldOverride2026 &operator=(const FieldOverride2026 &) = delete;

        FieldOverride2026(FieldOverride2026 &&) = default;
        FieldOverride2026 &operator=(FieldOverride2026 &&) = default;

    private:
        void createGoalShape(b2Vec2 center);
    };
} // namespace whiplash