#ifndef PROCEDURAL_FEEDER_H
#define PROCEDURAL_FEEDER_H

#include <regex>
#include <string>
#include <vector>
#include "procedural/core/Types/Fact.h"

namespace procedural {

class Feeder
{
public:
    Feeder();
    Fact* parse(const std::string& fact);
private:
    std::regex const pattern;


};

} // procedural

#endif //PROCEDURAL_FEEDER_H
