#ifndef PROCEDURAL_PATTERNRECOGNITION_H
#define PROCEDURAL_PATTERNRECOGNITION_H

#include <vector>
#include "procedural/core/Types/FactPattern.h"

namespace procedural {
struct PatternRecognition_t
{
    std::vector<std::vector<FactPattern>> patterns;
    std::vector<std::string> descriptions;


};

} //procedural
#endif //PROCEDURAL_PATTERNRECOGNITION_H
