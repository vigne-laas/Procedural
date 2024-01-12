#include "procedural/core/Graph/State.h"

#include <iostream>
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>
#include "algorithm"
namespace procedural {

State::State(const std::string& name, int id) : id_(id),
                                                name_(name),
                                                initial_node_(false),
                                                has_timeout_transition(false) {}

State* State::evolve(Fact* fact)
{
    for (auto& pair: next_facts_)
        if (pair.first.match(fact))
            return pair.second;

    return nullptr;
}

std::pair<State*, TransitionAction*> State::evolve(StateMachine* state_machine)
{
    for (auto& pair: next_actions_)
        if (pair.first.match(state_machine))
            return std::make_pair(pair.second, &pair.first);

    return std::make_pair(nullptr, nullptr);
}

//State* State::evolve(Action* action)
//{
//    for (auto& pair: next_tasks_)
//        if (pair.first.match(action))
//            return pair.second;
//
//    return nullptr;
//}

State* State::evolve(Task* task)
{
    for (auto& pair: next_tasks_)
        if (pair.first.match(task))
            return pair.second;

    return nullptr;
}

void State::addTransition(const TransitionFact& transition, State* next_state)
{
    next_facts_.emplace_back(transition, next_state);
}

void State::addTransition(const TransitionAction& transition, State* next_state)
{
    next_actions_.emplace_back(transition, next_state);
}

//void State::addTransition(const TransitionActionMethod& transition, State* next_state)
//{
//    next_actions_methods_.emplace_back(transition, next_state);
//}

void State::addTransition(const TransitionTask& transition, State* next_state)
{
    next_tasks_.emplace_back(transition, next_state);
}

void State::linkVariables(std::map<std::string, Variable_t>& variables_)
{
//    std::cout << "link : " << name_ << std::endl;
    for (auto& pair: next_facts_)
        pair.first.linkVariables(variables_);
    for (auto& pair: next_actions_)
        pair.first.linkVariables(variables_);
    for (auto& pair: next_tasks_)
        pair.first.linkVariables(variables_);
//    for (auto& pair: next_actions_methods_)
//        pair.first.linkVariables(variables_);
}

void State::expandTransitions(onto::OntologyManipulator* onto_manipulator)
{
    for (auto& next: next_facts_)
        next.first.expandProperty(&(onto_manipulator->objectProperties));
//    for (auto& transition: next_actions_methods_)
//        transition.first.setOntologyClient(&(onto_manipulator->individuals));
    for (auto& transition: next_tasks_)
        transition.first.setOntologyClient(&(onto_manipulator->individuals));
}

std::string State::toString() const
{
    std::string msg = "State : " + name_ + '_' + std::to_string(id_) + "\n";
    msg += isFinalNode() ? "\tFinal Node \n" : "";
    msg += initial_node_ ? "\tInitial Node \n" : "";
    msg += "\tTransitions (" + std::to_string(
            next_facts_.size() + next_actions_.size() + next_tasks_.size()) + "):";
    msg += " [ ";
    if (!next_facts_.empty())
        for (auto& pair_transition_state: next_facts_)
            msg += "\t" + pair_transition_state.first.toString() + "\n";
    if (!next_actions_.empty())
        for (auto& pair_transition_state: next_actions_)
            msg += "\t" + pair_transition_state.first.toString() + "\n";
    if (!next_tasks_.empty())
        for (auto& pair_transition_state: next_tasks_)
            msg += "\t" + pair_transition_state.first.toString() + "\n";
//    if (!next_actions_methods_.empty())
//        for (auto& pair_transition_state: next_actions_methods_)
//            msg += "\t" + pair_transition_state.first.toString() + "\n";
    msg += "]";
    return msg;
}

void State::addTimeoutTransition(State* final_state)
{
    has_timeout_transition = true;
    final_state_ = final_state;
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
bool State::validateConstraints(const std::vector<int>& constrains)
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

    for (const auto& pair: parent->next_tasks_)
    {
//        std::cout << "task pair : " << pair.second->name_ << "  origin : " << origin->name_ << std::endl;
        if (pair.second == origin)
        {
            TransitionTask t(pair.first, final_state->getId());
            this->addTransition(t, final_state);
        }
    }


    for (const auto& pair: parent->next_actions_)
    {
//        std::cout << "action pair : " << pair.second->name_ << "  origin : " << origin->name_ << std::endl;
        if (pair.second == origin)
        {
            TransitionAction t(pair.first, final_state->getId());
            this->addTransition(t, final_state);

        }
    }

}
void State::addValidateConstraints(const std::unordered_set<int>& constrains)
{
    for (const auto constrain: constrains)
        valide_constrains_.insert(constrain);

}
State* State::doTimeoutTransition()
{
    return final_state_;
}


} // namespace procedural