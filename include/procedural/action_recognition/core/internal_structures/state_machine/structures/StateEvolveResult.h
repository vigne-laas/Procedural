#ifndef PROCEDURAL_STATEEVOLVERESULT_H
#define PROCEDURAL_STATEEVOLVERESULT_H
#include "procedural/utils/structures/EvolveResult.h"
#include <map>
#include <string>

namespace procedural
{
class State;
struct StateEvolveResult_t
{
    recognition::EvolveResult result = recognition::EvolveResult::NO_EVOLUTION;
    State* next_state;
    std::map<std::string, std::string> remap;

};
}
#endif //PROCEDURAL_STATEEVOLVERESULT_H
