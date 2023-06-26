#ifndef PROCEDURAL_INCOMPLETESTATEMACHINE_H
#define PROCEDURAL_INCOMPLETESTATEMACHINE_H

#include <map>
#include <string>


namespace procedural
{
class StateMachine;
struct IncompleteStateMachine_t
{
    IncompleteStateMachine_t(StateMachine* net, const std::map<std::string,std::string>& remap): state_machine_(net), remap_variables_(remap){};

    StateMachine * state_machine_;
    std::map<std::string,std::string> remap_variables_;
};
}
#endif //PROCEDURAL_INCOMPLETESTATEMACHINE_H
