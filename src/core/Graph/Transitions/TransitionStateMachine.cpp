#include "procedural/core/Graph/Transitions/TransitionStateMachine.h"
#include "procedural/core/Graph/StateMachine.h"

namespace procedural {

TransitionStateMachine::TransitionStateMachine(uint32_t type, const std::map<std::string, std::string>& remap_var) : type_(type),
                                                                                                                     remap_var_(
                                                                                                                   remap_var),
                                                                                                                     flag_(
                                                                                                                   false)
{
    for (const auto& pair: remap_var_)
        variables_.insert(std::make_pair(pair.second, nullptr));
}

void TransitionStateMachine::linkVariables(std::map<std::string, Variable_t>& variables)
{
    for (auto& pair: variables_)
        pair.second = &(variables.at(pair.first));
}

bool TransitionStateMachine::match(StateMachine* stateMachine)
{
    if (stateMachine->getType() == type_)
    {
        for (const auto& pair: remap_var_)
        {
            uint32_t local_val = variables_.at(pair.second)->getValue();
            if (local_val == 0)
                variables_.at(pair.second)->value = stateMachine->getVar(pair.first).getValue();
            else if ((stateMachine->getVar(pair.first).getValue() != 0) && (stateMachine->getVar(pair.first).getValue() != local_val))
                return false;
        }
        flag_ = true;
        return true;
    } else
        return false;
}

std::string TransitionStateMachine::toString() const
{
    std::string res =
            "State Machine Transitions type : " + std::to_string(type_) + "(" + StateMachine::types_table.get(type_) + ")\n";
    for (auto& pair: remap_var_)
        res += "\t" + pair.first + " ==> " + pair.second + "\n";
    res += "Variables : \n";
    for (auto& var: variables_)
        res += "\t" + var.first + " : " + var.second->toString() + "\n";
    return res;
}

} // namespace procedural