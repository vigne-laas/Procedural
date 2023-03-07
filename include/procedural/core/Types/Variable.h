#ifndef PROCEDURAL_VARIABLE_H
#define PROCEDURAL_VARIABLE_H

#include <string>

namespace procedural {

struct Variable_t
{
    explicit Variable_t(const std::string& name) : literal(name), value(0){}

    std::string literal;
    uint32_t value;

    uint32_t getValue() const
    {
        return value;
    }

    std::string toString() const
    {
        return (value ? std::to_string(value) : literal);
    }
};

} // namespace procedural

#endif //PROCEDURAL_VARIABLE_H
