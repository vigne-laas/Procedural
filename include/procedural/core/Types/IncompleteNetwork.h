#ifndef PROCEDURAL_INCOMPLETENETWORK_H
#define PROCEDURAL_INCOMPLETENETWORK_H

#include <map>
#include <string>


namespace procedural
{
class StateMachine;
struct IncompleteNetwork_t
{
    IncompleteNetwork_t(StateMachine* net, const std::map<std::string,std::string>& remap): network_(net), remap_variables_(remap){};

    StateMachine * network_;
    std::map<std::string,std::string> remap_variables_;
};
}
#endif //PROCEDURAL_INCOMPLETENETWORK_H
