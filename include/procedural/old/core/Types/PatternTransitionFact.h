#ifndef PROCEDURAL_PATTERNTRANSITIONFACT_H
#define PROCEDURAL_PATTERNTRANSITIONFACT_H

#include "procedural/old/core/Types/PatternFact.h"

namespace procedural {

struct PatternTransitionFact_t
{
    PatternTransitionFact_t(int origin, PatternFact* fact, int next)
    {
        origin_state = origin;
        this->fact = fact;
        next_state = next;
    }

    int origin_state;
    PatternFact* fact;
    int next_state;
};

} // namespace procedural

#endif // PROCEDURAL_PATTERNTRANSITIONFACT_H