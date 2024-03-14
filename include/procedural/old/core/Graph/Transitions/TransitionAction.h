#ifndef PROCEDURAL_TRANSITIONSTATEMACHINE_H
#define PROCEDURAL_TRANSITIONSTATEMACHINE_H

#include <string>
#include <map>

#include "procedural/old/core/Types/Variable.h"

namespace procedural {

class StateMachine;

class TransitionAction
{
public:
    TransitionAction(uint32_t type, int next_state, const std::map<std::string, std::string>& remap_var,
                     const std::map<std::string, std::string>& arg_var = {});
    TransitionAction(const TransitionAction& transition, int id_next_state);
    void linkVariables(std::map<std::string, Variable_t>& variables);

    bool match(StateMachine* stateMachine);

    bool checkMatch() { return flag_; }
    std::map<std::string, std::string> getRemap() { return remap_var_; }

    std::string toString() const;
    std::string toShortString() const;

    bool operator<(const TransitionAction other) const;

    uint32_t getType() const { return type_; };

private:
    uint32_t type_;
    int id_next_state_;
    bool flag_;
    std::map<std::string, std::string> remap_var_;
    std::map<std::string, std::string> arg_var_;
    std::map<std::string, Variable_t*> variables_;
};

} // namespace procedural

#endif // PROCEDURAL_TRANSITIONSTATEMACHINE_H
