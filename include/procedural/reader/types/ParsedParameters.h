#ifndef PROCEDURAL_PARSEDPARAMETERS_H
#define PROCEDURAL_PARSEDPARAMETERS_H

#include <ostream>

namespace procedural {

struct ParsedParameters_t
{
    ParsedParameters_t(): ttl(0) {}

    bool empty() const { return ttl==0; }
    double ttl;

    friend std::ostream& operator<<(std::ostream& os, const ParsedParameters_t& lhs)
    {
        if(lhs.empty())
            return os;
        os << "ttl : " << lhs.ttl << "\n";
        return os;
    }
};

} // namespace procedural

#endif //PROCEDURAL_PARSEDPARAMETERS_H
