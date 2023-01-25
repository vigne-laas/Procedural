//
// Created by avigne on 25/01/23.
//

#ifndef PROCEDURAL_VARIABLE_H
#define PROCEDURAL_VARIABLE_H

#include <string>

namespace procedural {
struct Variable_t
{
    explicit Variable_t(const std::string& name){
        literal = name;
    }
    std::string literal;
    uint32_t value = 0;

    bool asValue() const
    {
        return value;
    }

    std::string toString() const
    {
        return (this->asValue() ? std::to_string(value) : literal);
    }


};
}


#endif //PROCEDURAL_VARIABLE_H
