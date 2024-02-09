#include "procedural/core/Graph/Transitions/TransitionAction.h"
#include "procedural/core/Graph/StateMachine.h"

namespace procedural {

TransitionAction::TransitionAction(uint32_t type, int next_state, const std::map<std::string, std::string>& remap_var,
                                   const std::map<std::string, std::string>& arg_var) :
        type_(type),
        remap_var_(remap_var),
        arg_var_(arg_var),
        flag_(false),
        id_next_state_(next_state)
{
    for (const auto& pair: remap_var_)
        variables_.insert(std::make_pair(pair.second, nullptr));
    for (const auto& pair: arg_var_)
        variables_.insert(std::make_pair(pair.first, nullptr));

}

TransitionAction::TransitionAction(const TransitionAction& transition, int id_next_state) : type_(transition.type_),
                                                                                            flag_(transition.flag_),
                                                                                            id_next_state_(
                                                                                                    id_next_state),
                                                                                            remap_var_(
                                                                                                    transition.remap_var_),
                                                                                            arg_var_(
                                                                                                    transition.arg_var_)
{
    for (const auto& pair: remap_var_)
        variables_.insert(std::make_pair(pair.second, nullptr));
    for (const auto& pair: arg_var_)
        variables_.insert(std::make_pair(pair.first, nullptr));

}

void TransitionAction::linkVariables(std::map<std::string, Variable_t>& variables)
{
//    std::cout << "link variables action : " << std::endl;
    for (auto& pair: variables_)
    {
//        std::cout << "var : " << pair.first << " count : " << std::to_string(variables.count(pair.first)) << std::endl;
//        if (variables.count(pair.first) > 0)
        pair.second = &(variables.at(pair.first));
//        else
//            std::cout << "variables not found : " << pair.first << std::endl;
    }
}

bool TransitionAction::match(StateMachine* stateMachine)
{
    if (stateMachine->getType() == type_)
    {
        for (const auto& pair: remap_var_)
        {
            uint32_t local_val = variables_.at(pair.second)->getValue();
            if (local_val == 0)
                variables_.at(pair.second)->value = stateMachine->getVar(pair.first).getValue();
            else if ((stateMachine->getVar(pair.first).getValue() != 0) &&
                     (stateMachine->getVar(pair.first).getValue() != local_val))
                return false;
        }
        flag_ = true;
        return true;
    } else
        return false;
}

std::string TransitionAction::toString() const
{
    std::string res =
            "State Machine Transitions type : " + std::to_string(type_) + "(" + StateMachine::types_table.get(type_) +
            ") to " + std::to_string(id_next_state_) + "\n";
    for (auto& pair: remap_var_)
        res += "\t" + pair.first + " ==> " + pair.second + "\n";
    res += "Variables : \n";
    for (auto& var: variables_)
        res += "\t" + var.first + " : " + var.second->toString() + "\n";
    return res;
}

std::string TransitionAction::toShortString() const
{
    std::string res =
            "State Machine Transitions type : " + std::to_string(type_) + "(" + StateMachine::types_table.get(type_) +
            ") to " + std::to_string(id_next_state_) + "\n";
    res += "\t Remap : ";
    for (auto& pair: remap_var_)
        res += "\t" + pair.first + " ==> " + pair.second + "\n";
    if (!variables_.empty())
    {
        res += "\t Vars : ";
        for (auto it = variables_.begin(); it != variables_.end(); ++it)
            res += "\t" + it->first + (std::next(it) != variables_.end() ? "," : "");
    }
    return res;
}
bool TransitionAction::operator<(const TransitionAction other) const
{
    return id_next_state_ < other.id_next_state_;
}


} // namespace procedural