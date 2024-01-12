#include "procedural/core/Graph/State.h"

#include <iostream>
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>
#include <fstream>
#include <filesystem>
#include "algorithm"
namespace procedural {

State::State(const std::string& name, int id) : id_(id),
                                                name_(name),
                                                initial_node_(false),
                                                has_timeout_transition(false), level_(id / 10) {}

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

std::string State::toShortString() const
{
    std::string msg = "State : " + name_ + '_' + std::to_string(id_) + " lvl : " + std::to_string(level_) + "\n";
    msg += isFinalNode() ? "\tFinal Node \n" : "";
    msg += initial_node_ ? "\tInitial Node \n" : "";
    if (!valide_constrains_.empty())
    {
        msg += "Const : ";
        for (auto it = valide_constrains_.begin(); it != valide_constrains_.end(); it++)
        {
            msg += std::to_string(*it);
            if (std::next(it) != valide_constrains_.end())
                msg += ",";
        }

    }
    if (!parents_.empty())
    {
        msg += "\tparents : ";
        for (auto it = parents_.begin(); it != parents_.end(); it++)
        {
            msg += std::to_string((*it)->id_);
            if (std::next(it) != parents_.end())
                msg += ",";
        }

    }

    return msg;
}

void State::addTimeoutTransition(State* final_state)
{
    has_timeout_transition = true;
    final_state_ = final_state;
}
void State::addParents(State* parent_state)
{
    parents_.insert(parent_state);
}

void State::addValidateConstraints(const std::vector<int>& constrains)
{
    for (const auto& constrain: constrains)
    {
//        std::cout << "Add constraint vector :" << std::to_string(constrain) << " to : " << name_ << std::endl;
        valide_constrains_.insert(constrain);
    }

}

void State::addValidateConstraints(int id)
{
//    std::cout << "Add constraint :" << std::to_string(id) << " to : " << name_ << std::endl;
    valide_constrains_.insert(id);
}
bool State::validateConstraints(const std::vector<int>& constrains)
{
    return std::all_of(constrains.begin(), constrains.end(), [this](int val) {
        return valide_constrains_.find(val) != valide_constrains_.end();
    });
}

bool State::validateConstraints(const std::unordered_set<int>& constraints) const
{
    return std::all_of(constraints.begin(), constraints.end(), [this](int val) {
        return valide_constrains_.find(val) != valide_constrains_.end();
    });
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
            final_state->addParents(this);
            final_state->addValidateConstraints(getConstrains_());
        }
    }


    for (const auto& pair: parent->next_actions_)
    {
//        std::cout << "action pair : " << pair.second->name_ << "  origin : " << origin->name_ << std::endl;
        if (pair.second == origin)
        {
            TransitionAction t(pair.first, final_state->getId());
            this->addTransition(t, final_state);
            final_state->addParents(this);
            final_state->addValidateConstraints(getConstrains_());


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

void State::generateDOT_Facts(std::ofstream& dotFile, std::set<int>& visitedStates) const
{
    for (const auto& transition: next_facts_)
    {


        dotFile << "  " << name_ + "_" + std::to_string(id_) << " -> "
                << transition.second->name_ + "_" + std::to_string(transition.second->id_) << " [label=\""
                << transition.first.toString() << "\", color=\"blue\"];" << std::endl;


        visitedStates.insert(id_);  // Mark the current state as visited

        if (visitedStates.find(transition.second->id_) == visitedStates.end())
        {
            transition.second->generateDOT_Facts(dotFile, visitedStates);
            transition.second->generateDOT_Actions(dotFile, visitedStates);
            transition.second->generateDOT_Tasks(dotFile, visitedStates);
        }
    }
}

void State::generateDOT_Actions(std::ofstream& dotFile, std::set<int>& visitedStates) const
{
    for (const auto& transition: next_actions_)
    {
        dotFile << "  " << name_ + "_" + std::to_string(id_) << " -> "
                << transition.second->name_ + "_" + std::to_string(transition.second->id_) << " [label=\""
                << transition.first.toShortString() << "\", color=\"green\"];" << std::endl;

        visitedStates.insert(id_);  // Mark the current state as visited

        if (visitedStates.find(transition.second->id_) == visitedStates.end())
        {
            transition.second->generateDOT_Facts(dotFile, visitedStates);
            transition.second->generateDOT_Actions(dotFile, visitedStates);
            transition.second->generateDOT_Tasks(dotFile, visitedStates);
        }
    }
}

void State::generateDOT_Tasks(std::ofstream& dotFile, std::set<int>& visitedStates) const
{
    for (const auto& transition: next_tasks_)
    {

        dotFile << "  " << name_ + "_" + std::to_string(id_) << " -> "
                << transition.second->name_ + "_" + std::to_string(transition.second->id_)
                << " [label=\"" << transition.first.toShortString() << "\", color=\"red\"];" << std::endl;

        visitedStates.insert(id_);  // Mark the current state as visited

        if (visitedStates.find(transition.second->id_) == visitedStates.end())
        {
            transition.second->generateDOT_Facts(dotFile, visitedStates);
            transition.second->generateDOT_Actions(dotFile, visitedStates);
            transition.second->generateDOT_Tasks(dotFile, visitedStates);
        }
    }
}

void State::saveDOTFile(std::ofstream& dot_file) const
{

    // Use a set to track visited states
    std::set<int> visitedStates;

    // Call specific functions to generate DOT content for each type of transition
    generateDOT_Facts(dot_file, visitedStates);
    generateDOT_Actions(dot_file, visitedStates);
    generateDOT_Tasks(dot_file, visitedStates);

}
std::map<State*, Transitions_t> State::getValideParents(std::vector<int> constraints)
{
    std::map<State*, Transitions_t> res;
    for (auto parent: getParents_())
    {
        if (parent->validateConstraints(constraints))
        {
            for (const auto& pair: parent->next_tasks_)
            {
                if (pair.second == this)
                {
                    auto key = res.find(parent);
                    if (key != res.end())
                    {
                        key->second.next_tasks_.insert(pair.first);
                    } else
                    {
                        Transitions_t T;
                        T.next_tasks_.insert(pair.first);
                        res.insert(std::make_pair(parent, T));
                    }
                }
            }

            for (const auto& pair: parent->next_actions_)
            {
                if (pair.second == this)
                {
                    auto key = res.find(parent);
                    if (key != res.end())
                    {
                        key->second.next_actions_.insert(pair.first);
                    } else
                    {
                        Transitions_t T;
                        T.next_actions_.insert(pair.first);
                        res.insert(std::make_pair(parent, T));
                    }
                }
            }
            parent->getValideParents(constraints, res, this);
        }
    }
    return res;
}
std::map<State*, Transitions_t>
State::getValideParents(std::vector<int> constraints, std::map<State*, Transitions_t>& map, State* origin)
{
//    std::cout << "in get valide parent 2 from state : " << name_ << "_" << id_ << std::endl;
    for (auto parent: getParents_())
    {
        if (parent->validateConstraints(constraints))
        {
            for (const auto& pair: parent->next_tasks_)
            {
                if (pair.second == origin)
                {
                    auto key = map.find(parent);
                    if (key != map.end())
                    {
                        key->second.next_tasks_.insert(pair.first);
                    } else
                    {
                        Transitions_t T;
                        T.next_tasks_.insert(pair.first);
                        map.insert(std::make_pair(parent, T));
                    }
                }
            }

            for (const auto& pair: parent->next_actions_)
            {
                if (pair.second == origin)
                {
                    auto key = map.find(parent);
                    if (key != map.end())
                    {
                        key->second.next_actions_.insert(pair.first);
                    } else
                    {
                        Transitions_t T;
                        T.next_actions_.insert(pair.first);
                        map.insert(std::make_pair(parent, T));
                    }

                }
            }
            parent->getValideParents(constraints, map, this);
        }
    }
    if (parents_.empty())
    {
        if (validateConstraints(constraints))
        {
            for (const auto& pair: next_tasks_)
            {
                if (pair.second == origin)
                {
                    auto key = map.find(this);
                    if (key != map.end())
                    {
                        key->second.next_tasks_.insert(pair.first);
                    } else
                    {
                        Transitions_t T;
                        T.next_tasks_.insert(pair.first);
                        map.insert(std::make_pair(this, T));
                    }
                }
            }

            for (const auto& pair: next_actions_)
            {
                if (pair.second == origin)
                {
                    auto key = map.find(this);
                    if (key != map.end())
                    {
                        key->second.next_actions_.insert(pair.first);
                    } else
                    {
                        Transitions_t T;
                        T.next_actions_.insert(pair.first);
                        map.insert(std::make_pair(this, T));
                    }

                }
            }

        }

    }
    return map;
}
void State::closeTo(std::vector<State*> possible_states, Transitions_t transitions)
{
//    std::cout << "current level : " << getLevel() << "try to close to next level : " << getLevel() + 1 << std::endl;
    for (const auto& next_task: transitions.next_tasks_)
    {
        auto temp_constraint = this->getConstrains_();
        temp_constraint.insert((int) next_task.getTaskId());
//        std::cout << "Task  ensemble des contraintes a valider : ";
//        for (auto id: temp_constraint)
//            std::cout << id << ",";
//        std::cout << std::endl;

        for (auto state: possible_states)
        {
//            std::cout << "possible state :" << state->name_ << "_" << state->id_ << std::endl;
            if (state->validateConstraints(temp_constraint))
            {
//                std::cout << "Valide state : " << state->name_ << "_" << state->id_ << std::endl;
                TransitionTask t(next_task, state->getId());
//                std::cout << "add transition betwenn " << this->name_ << "_" << this->id_ << "and : "
//                          << state->getFullName() << std::endl;
                this->addTransition(t, state);
                state->addParents(this);
            }
        }
    }

    for (const auto& next_actions: transitions.next_actions_)
    {
        auto temp_constraint = this->getConstrains_();
        temp_constraint.insert((int) next_actions.getType());
//        std::cout << "action  ensemble des contraintes a valider : ";
//        for (auto id: temp_constraint)
//            std::cout << id << ",";
//        std::cout << std::endl;
        for (auto state: possible_states)
        {
//            std::cout << "possible state :" << state->name_ << "_" << state->id_ << std::endl;
            if (state->validateConstraints(temp_constraint))
            {
//                std::cout << "Valide state : " << state->name_ << "_" << state->id_ << std::endl;
                TransitionAction t(next_actions, state->getId());
//                std::cout << "add transition betwenn " << this->name_ << "_" << this->id_ << "and : "
//                          << state->getFullName() << std::endl;
                this->addTransition(t, state);
                state->addParents(this);
            }
        }


    }


}


} // namespace procedural