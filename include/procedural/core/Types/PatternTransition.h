#include "procedural/core/Types/FactPattern.h"

namespace procedural 
{

struct PatternTransition_t
{
    PatternTransition_t(int origin, FactPattern* fact, int next, bool initial)
    {
        origin_state = origin;
        this->fact = fact;
        next_state = next;
        is_initial_state = initial;
    }

    bool is_initial_state;
    int origin_state;
    FactPattern* fact;
    int next_state;
};


} // namespace procedural
