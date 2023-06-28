#include <iostream>
#include <set>
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>
#include "procedural/core/Graph/StateMachine.h"
#include "procedural/core/Graph/TransitionStateMachine.h"

namespace procedural {

WordTable StateMachine::types_table;

StateMachine::StateMachine(const std::string& name, int id, uint32_t level) : type_str_(name),
                                                                              id_(id),
                                                                              closed_(false),
                                                                              valid_(false),
                                                                              age_(0, 0),
                                                                              last_update_(0, 0),
                                                                              level_(level), new_explanations_(false)
{
    full_name_ = type_str_ + "_" + std::to_string(id);
    variables_.emplace("self", full_name_);
    type_ = StateMachine::types_table.get(type_str_);
}

bool StateMachine::evolve(Fact* fact)
{
    if ((valid_ && closed_) == false)
        return false;
    auto evolution = current_state_->evolve(fact);

    if (evolution == nullptr)
        return false;
    if (current_state_->getId() == id_initial_state_)
        age_ = fact->getTimeStamp();
    last_update_ = fact->getTimeStamp();

    current_state_ = evolution;
    id_facts_involve.push_back(fact->getId()); // prepare to id on facts.
    new_explanations_ = checkIncompletsStateMachines();
    return true;
}

bool StateMachine::evolve(StateMachine* stateMachine)
{
    if (level_ < stateMachine->getLevel())
        return false;
    if ((valid_ && closed_) == false)
        return false;
    auto evolution = current_state_->evolve(stateMachine);
    if (evolution.first == nullptr)
        return false;
    if (current_state_->getId() == id_initial_state_ || age_ > stateMachine->getAge())
        age_ = stateMachine->getAge();
    last_update_ = stateMachine->getLastupdate();

    for (auto id_fact: stateMachine->getIdsFacts())
        id_facts_involve.push_back(id_fact);
    if (stateMachine->getCompletionRatio() != 1.0)
        incompletes_state_machines_.emplace_back(stateMachine, evolution.second->getRemap());

    current_state_ = evolution.first;
    new_explanations_ = checkIncompletsStateMachines();
//    std::cout << "new_explanations_  : " << new_explanations_ <<std::endl;
    return true;
}


float StateMachine::getCompletionRatio() const
{
    float incomplete = 0;
    for (const auto& var: variables_)
        if (var.second.getValue() == 0 && var.first != "self")
            incomplete++;
    return 1 - (incomplete / float(variables_.size()));
}


std::vector<std::string> StateMachine::getLiteralVariables()
{
    std::vector<std::string> res;
    for (auto const& map_elemet: variables_)
        res.push_back(map_elemet.first);
    return res;
}


bool StateMachine::addTransition(const PatternTransitionFact_t& transitionFact)
{
    if (closed_ == false)
    {
        addState(transitionFact.origin_state);
        addState(transitionFact.next_state);

        insertVariable(transitionFact.fact->getVarSubject());
        insertVariable(transitionFact.fact->getVarObject());

        TransitionFact t = TransitionFact(*(transitionFact.fact));
        states_[transitionFact.origin_state]->addTransition(t, states_[transitionFact.next_state]);
        return true;
    } else
        return false;

}

bool StateMachine::addStateMachine(const PatternTransitionStateMachine_t& transitionStateMachine)
{
    if (closed_ == false)
    {
        addState(transitionStateMachine.origin_);
        addState(transitionStateMachine.next_);
        for (auto& var: transitionStateMachine.remap_var_)
            insertVariable(var.second);

        auto type = StateMachine::types_table.get(transitionStateMachine.type_);
        TransitionStateMachine transition(type, transitionStateMachine.remap_var_);
        states_[transitionStateMachine.origin_]->addTransition(transition, states_[transitionStateMachine.next_]);
        return true;
    } else
        return false;
}

bool StateMachine::addDescription(const ActionDescription_t& des)
{
    descriptions_.emplace_back(des, variables_);
    return true;
}

bool StateMachine::closeStateMachine()
{
    linkStateMachine();
    closed_ = true;
    processInitialState();
    valid_ = true;
    return valid_;
}

std::string StateMachine::toString()
{
    std::string res;
    for (auto& state: states_)
    {
        if (res != "")
            res += "\n";
        res += "id : " + std::to_string(state.first) + " " + state.second->toString();
    }
    res += "\n\n current state : " + current_state_->toString();
    return res;
}

StateMachine* StateMachine::clone(int new_id, int last_state_required)
{
    if ((valid_ && closed_) == false)
        return nullptr;

    StateMachine* N = new StateMachine(type_str_, new_id);
    N->variables_ = variables_;
    N->variables_.at("self").literal = N->getName();
    N->descriptions_ = descriptions_;

    for (auto& state: states_)
        N->addState(state.first);

    for (auto& state: N->states_)
    {
        for (auto& pair_transition: states_.at(state.first)->getNextsFacts())
        {
            TransitionFact t = pair_transition.first;
            state.second->addTransition(t, N->states_.at(pair_transition.second->getId()));
        }
        for (auto& pair_transition: states_.at(state.first)->getNextsStateMachines())
        {
            TransitionStateMachine t = pair_transition.first;
            state.second->addTransition(t, N->states_.at(pair_transition.second->getId()));
        }
    }
    N->addTimeoutTransition(last_state_required);
    N->closeStateMachine();
    return N;
}

void StateMachine::displayVariables()
{
    for (auto& var: variables_)
        std::cout << "key : " << var.first << " => " << var.second.toString() << std::endl;
}

std::string StateMachine::describe(bool expl)
{
    if ((valid_ && closed_) == false)
        return "";
    std::string msg = "\t";
    for (auto& description: descriptions_)
    {
        if (expl)
            msg += description.explainExplicit() + " / ";
        else
            msg += description.explain() + " / ";
    }
    msg += "// completion ratio :" + std::to_string(getCompletionRatio());

    if (incompletes_state_machines_.empty())
        return msg;

    msg += "\t // incomplete State Machine link : ";
    for (const auto& incomplete_net: incompletes_state_machines_)
    {
        msg += incomplete_net.state_machine_->getName();
        msg += "\n\t new level : " + std::to_string(incomplete_net.state_machine_->getCompletionRatio()) + "\n";
//         net.first->displayVariables();
    }


    return msg;
}

bool StateMachine::involveFacts(const std::set<uint32_t>& facts)
{
    for (auto id_fact: id_facts_involve)
        if (facts.find(id_fact) == facts.end())
            return false;
    return true;
}

void StateMachine::addTimeoutTransition(int last_state_required)
{
    states_.at(last_state_required)->addTimeoutTransition();
}


/* ------------------------------ private part ------------------------------ */

bool StateMachine::checkIncompletsStateMachines()
{
    updated_sub_state_machines_.clear();
    if (incompletes_state_machines_.empty())
        return false;
    for (auto& incomplete_net: incompletes_state_machines_)
        if (incomplete_net.state_machine_->updateVar(incomplete_net.remap_variables_, variables_))
            updated_sub_state_machines_.push_back(incomplete_net.state_machine_);

    return updated_sub_state_machines_.empty() == false;
}


void StateMachine::addState(int id_state)
{
    if (states_.find(id_state) == states_.end())
        states_.emplace(id_state, new State(getName(), id_state));
}

void StateMachine::linkStateMachine()
{
    for (auto& state: states_)
    {
        state.second->linkVariables(variables_);
        //state.expandTransitions();
    }
    for (auto& des: descriptions_)
        des.linkVariables(variables_);
}

void StateMachine::insertVariable(const std::string& variable)
{
    variables_.emplace(variable, variable);
}


void StateMachine::processInitialState()
{
    std::unordered_set<uint32_t> id_states_nexts;
    for (auto& pair_states: states_)
    {
        for (auto& nexts_state: pair_states.second->getNextsFacts())
            id_states_nexts.insert(nexts_state.second->getId());
        for (auto& nexts_state: pair_states.second->getNextsStateMachines())
            id_states_nexts.insert(nexts_state.second->getId());
    }

    std::unordered_set<int> result;
    for (auto& state: states_)
    {
        if (id_states_nexts.find(state.first) == id_states_nexts.end())
            result.insert(state.first);
    }

    int nb_initial_state = result.size();
    if (result.size() == 0)
    {
        throw NoInitialStateStateMachineException();
    } else if (nb_initial_state > 1)
    {
        // Maybe raise an error rather than printing text
        std::unordered_set<State*> invalid_states;
        for (auto res: result)
        {
            invalid_states.insert(states_.at(res));
        }
        throw MultiInitialStateStateMachineException(invalid_states);
    } else
    {
        id_initial_state_ = *result.begin();
        current_state_ = states_.at(id_initial_state_);
    }
}
bool
StateMachine::updateVar(const std::map<std::string, std::string>& remap, const std::map<std::string, Variable_t>& new_var)
{
    bool res = false;
    for (const auto& pair: remap)
    {
        if (variables_.at(pair.first).getValue() == 0 && new_var.at(pair.second).getValue() != 0)
        {
            variables_.at(pair.first).value = new_var.at(pair.second).getValue();
            res = true;
        }
    }
    return res;
}
void StateMachine::displayTypesTable()
{
    for (int index = 0; index < StateMachine::types_table.size(); index++)
        std::cout << " types_table[" << index << "] : " << types_table[index] << std::endl;
}
void StateMachine::setId(int new_id)
{
    id_ = new_id;
    full_name_ = type_str_ + "_" + std::to_string(new_id);
    variables_.at("self").literal = full_name_;
//    std::cout << "full_name :" << full_name_ << std::endl;
//    std::cout << "description : " << descriptions_.at(0).var_subject_str_ << " isA "
//              << descriptions_.at(0).var_object_str_ << std::endl;

}
std::string StateMachine::getStrStructure()
{
    std::string res;
    for (auto& state: states_)
    {
        if (res != "")
            res += "\n";
        res += "id : " + std::to_string(state.first) + " " + state.second->toString();
    }
    return res;
}
void StateMachine::expandProperties(ObjectPropertyClient* object_client)
{
    for (auto& state: states_)
        (state.second)->expandTransitions(object_client);

}


} // namespace procedural