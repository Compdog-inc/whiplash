#include "whiplash/globalfield.h"

namespace whiplash
{
    static FieldOverride2026 globalField;

    FieldOverride2026 &getField()
    {
        return globalField;
    }
} // namespace whiplash