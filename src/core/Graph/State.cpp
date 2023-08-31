#include "procedural/core/Graph/State.h"

#include <iostream>
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>
#include "algorithm"
namespace procedural {

State::State(const std::string& name, int id) : id_(id),
                                                name_(name + "_" + std::to_string(id)),
                                                initial_node_(false),
                                                has_timeout_transition(false)
{}

State* State::evolve(Fact* fact)
{
    for (auto& pair: nexts_facts_)
        if (pair.first.match(fact))
            return pair.second;

    return nullptr;
}

std::pair<State*, TransitionStateMachine*> State::evolve(StateMachine* state_machine)
{
    for (auto& pair: nexts_state_machines_)
        if (pair.first.match(state_machine))
            return std::make_pair(pair.second, &pair.first);

    return std::make_pair(nullptr, nullptr);
}

State* State::evolve(Action* fact)
{
    for (auto& pair: nexts_actions_)
        if (pair.first.match(fact))
            return pair.second;

    return nullptr;
}

State* State::evolve(Task* fact)
{
    for (auto& pair: nexts_tasks_)
        if (pair.first.match(fact))
            return pair.second;

    return nullptr;
}

void State::addTransition(const TransitionFact& transition, State* next_state)
{
    nexts_facts_.emplace_back(transition, next_state);
}

void State::addTransition(const TransitionStateMachine& transition, State* next_state)
{
    nexts_state_machines_.emplace_back(transition, next_state);
}

void State::addTransition(const TransitionActionMethod& transition, State* next_states)
{
    nexts_actions_.emplace_back(transition, next_states);
}

void State::addTransition(const TransitionTask& transition, State* next_states)
{
    nexts_tasks_.emplace_back(transition, next_states);
}

void State::linkVariables(std::map<std::string, Variable_t>& variables_)
{
    for (auto& pair: nexts_facts_)
        pair.first.linkVariables(variables_);
    for (auto& pair: nexts_state_machines_)
        pair.first.linkVariables(variables_);
    for (auto& pair: nexts_tasks_)
        pair.first.linkVariables(variables_);
    for (auto& pair: nexts_actions_)
        pair.first.linkVariables(variables_);
}

void State::expandTransitions(onto::OntologyManipulator* onto_manipulator)
{
    for (auto& next: nexts_facts_)
        next.first.expandProperty(&(onto_manipulator->objectProperties));
    for (auto& transition: nexts_actions_)
        transition.first.setOntologyClient(&(onto_manipulator->individuals));
    for (auto& transition: nexts_tasks_)
        transition.first.setOntologyClient(&(onto_manipulator->individuals));
}

std::string State::toString() const
{
    std::string msg = "State : " + name_ + "\n";
    msg += isFinalNode() ? "\tFinal Node \n" : "";
    msg += initial_node_ ? "\tInitial Node \n" : "";
    msg += "\tTransitions (" + std::to_string(
            nexts_facts_.size() + nexts_state_machines_.size() + nexts_tasks_.size() + nexts_actions_.size()) + "):";
    msg += " [ ";
    if (!nexts_facts_.empty())
        for (auto& pair_transition_state: nexts_facts_)
            msg += "\t" + pair_transition_state.first.toString() + "\n";
    if (!nexts_state_machines_.empty())
        for (auto& pair_transition_state: nexts_state_machines_)
            msg += "\t" + pair_transition_state.first.toString() + "\n";
    if (!nexts_tasks_.empty())
        for (auto& pair_transition_state: nexts_tasks_)
            msg += "\t" + pair_transition_state.first.toString() + "\n";
    if (!nexts_actions_.empty())
        for (auto& pair_transition_state: nexts_actions_)
            msg += "\t" + pair_transition_state.first.toString() + "\n";
    msg += "]";
    return msg;
}

void State::addTimeoutTransition()
{
    has_timeout_transition = true;
}
void State::addParents(State* parent_state)
{
    parents_.emplace_back(parent_state);
}

void State::addValidateConstraints(const std::vector<int>& constrains)
{
    for (const auto& constrain: constrains)
        valide_constrains_.insert(constrain);
}
bool State::valideConstrains(const std::vector<int>& constrains)
{
    if (std::all_of(constrains.cbegin(), constrains.cend(), [this](int val) {
        auto res = valide_constrains_.find(val);
        return res != valide_constrains_.end();
    }))
        return true;
    return false;
}
void State::closeTo(State* final_state, State* parent, State* origin)
{
//    std::cout << "ids : final " << std::to_string(final_state->getId()) << " parent : "
//              << std::to_string(parent->getId()) << " origin : " << std::to_string(origin->getId()) << "  this : "
//              << this->getId() << std::endl;

    for (const auto& pair: parent->nexts_tasks_)
    {
//        std::cout << "task pair : " << pair.second->name_ << "  origin : " << origin->name_ << std::endl;
        if (pair.second == origin)
        {
            TransitionTask t(pair.first,final_state->getId());
            this->addTransition(t, final_state);
        }
    }


    for (const auto& pair: parent->nexts_actions_)
    {
//        std::cout << "action pair : " << pair.second->name_ << "  origin : " << origin->name_ << std::endl;
        if (pair.second == origin)
        {
            TransitionActionMethod t(pair.first, final_state->getId());
            this->addTransition(t, final_state);

        }
    }

//        std::cout << "into close to " << pair.first.toString() << "state :" << pair.second << std::endl;
//
//
//    auto val = std::find_if(parent->getNextsTasks().cbegin(), parent->getNextsTasks().cend(),
//                            [origin](const std::pair<TransitionTask, State*>& pair) {
//                                if (pair.second != nullptr)
//                                    return pair.second == origin;
//                                else
//                                    return false;
//                            });
//    if (val != parent->getNextsTasks().end())
//    {
//        this->addTransition(val->first, final_state);
//        return;
//
//    }


//    auto val_action = std::find_if(parent->getNextsActions().begin(), parent->getNextsActions().end(),
//                                   [origin](const std::pair<TransitionActionMethod, State*>& pair) {
//                                       return pair.second == origin;
//                                   });
//    if (val_action != parent->getNextsActions().end())
//    {
//        this->addTransition(val_action->first, final_state);
//        return;
//    }


}
void State::addValidateConstraints(const std::unordered_set<int>& constrains)
{
    for (const auto constrain: constrains)
        valide_constrains_.insert(constrain);

}


} // namespace procedural