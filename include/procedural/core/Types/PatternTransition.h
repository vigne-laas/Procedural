#include "procedural/core/Types/FactPattern.h"

namespace procedural 
{

struct PatternTransition_t
{
    PatternTransition_t(int origin, FactPattern* fact, int next)
    {
        origin_state = origin;
        this->fact = fact;
        next_state = next;
    }

    int origin_state;
    FactPattern* fact;
    int next_state;
};


} // namespace procedural
