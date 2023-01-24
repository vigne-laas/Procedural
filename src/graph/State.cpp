#include "procedural/graph/State.h"

namespace procedural {
State::State(std::string name) : name_(name), id_(1),initial_node_(false),final_node_(false)
{

}

State* State::evolve(const Fact& fact) const
{
    for (auto transition: transitions_)
    {
        auto new_state = transition.evolve(fact);
        if (new_state != nullptr)
            return new_state;
    }
    return nullptr;
}

void State::addTransition(const Transition& transition)
{
    transitions_.push_back(transition);
}

void State::setTransition(const std::vector<Transition>& transitions)
{
    transitions_ = transitions;
}


void State::setInitialNode()
{
    initial_node_ = true;
}

void State::setFinalNode()
{
    final_node_ = true;
}

bool State::isInitialNode() const
{
    return initial_node_;
}

bool State::isFinalNode() const
{
    return transitions_.empty();
}

std::vector<Transition>& State::getTransitions()
{
    return transitions_;
}

std::string State::toString()
{
    std::string msg = "State : " + name_ + "_" + std::to_string(id_)+"\n";
    msg += isFinalNode() ? "Final Node \n" : "";
    msg += initial_node_ ? "Initial Node \n" : "";

    msg += "\t nombre de transition : " + std::to_string(transitions_.size()) + "\n";
    for (auto transition: transitions_)
    {
        msg += "\t\t -  " + transition.toString();
    }
    return msg;
}

} // procedural