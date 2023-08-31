#include "procedural/core/Graph/Transitions/TransitionAction.h"
#include "procedural/core/Types/Action.h"

namespace procedural {
TransitionAction::TransitionAction(uint32_t id_action, int next_state,
                                   const std::map<std::string, std::string>& arguments)
        : action_id_(id_action),
          id_next_state_(next_state), arguments_(arguments)
{
    for (const auto& arg: arguments)
        variables_.insert(std::make_pair(arg.first, nullptr));
}

TransitionAction::TransitionAction(const TransitionAction& transition, int id_next_state) :
        action_id_(transition.action_id_),
        id_next_state_(id_next_state),
        arguments_(transition.arguments_)
{
    for (const auto& arg: arguments_)
        variables_.insert(std::make_pair(arg.first, nullptr));
}

void TransitionAction::linkVariables(std::map<std::string, Variable_t>& variables)
{
    std::cout <<"link variables action : " << std::endl;
    for (auto& pair: variables_)
        pair.second = &(variables.at(pair.first));
}
bool TransitionAction::match(Action* action)
{
    if (action->getType() != action_id_)
        return false;
    for (auto action: action->getFinishedStateMachine())
        for (auto var_key: action->getLiteralVariables())

            return false;
    return true;
}
std::string TransitionAction::toString() const
{
    std::string res =
            "Action Transitions type : " + std::to_string(action_id_) + "(" + Action::action_types.get(action_id_) +
            ") to " + std::to_string(id_next_state_) + " \n";
    if(!variables_.empty())
    {
        res += "Variables : \n";
        for (auto& var: variables_)
            res += "\t" + var.first + " : " + var.second->toString() + "\n";
    }
    return res;


}
void TransitionAction::setOntologyClient(onto::IndividualClient* indiv_client)
{

}

} // procedural