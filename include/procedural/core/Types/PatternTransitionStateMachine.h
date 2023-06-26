#ifndef PROCEDURAL_PATTERNTRANSITIONSTATEMACHINE_H
#define PROCEDURAL_PATTERNTRANSITIONSTATEMACHINE_H

#include <map>
#include <string>

namespace procedural {

struct PatternTransitionStateMachine_t
{
    PatternTransitionStateMachine_t(int origin, int next,
                                    const std::string& state_machine_type,
                                    const std::map<std::string, std::string>& remap_var) : origin_(origin),
                                                                                      next_(next),
                                                                                      type_(state_machine_type),
                                                                                      remap_var_(remap_var)
    {}

    int origin_;
    int next_;
    std::string type_;
    std::map<std::string, std::string> remap_var_;
};

} // namespace procedural

#endif // PROCEDURAL_PATTERNTRANSITIONSTATEMACHINE_H
