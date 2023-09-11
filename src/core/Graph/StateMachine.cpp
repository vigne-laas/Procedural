#include <iostream>
#include <set>
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>
#include "procedural/core/Graph/StateMachine.h"
#include "procedural/core/Graph/Transitions/TransitionAction.h"

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

EvolveResult_t StateMachine::evolve(Fact* fact)
{
    EvolveResult_t res;
    if ((valid_ && closed_) == false)
        return res;
    auto evolution = current_state_->evolve(fact);

    if (evolution == nullptr)
        return res;
    if (current_state_->getId() == id_initial_state_)
        age_ = fact->getTimeStamp();
    last_update_ = fact->getTimeStamp();

    current_state_ = evolution;
    id_facts_involve.push_back(fact->getId());
    new_explanations_ = checkIncompletsStateMachines();
    res.update_available = new_explanations_;
    res.state = FeedResult::EVOLVE;
    if (current_state_->isFinalNode())
        res.state = FeedResult::COMPLETE;
    return res;
}

EvolveResult_t StateMachine::evolve(StateMachine* stateMachine)
{
    EvolveResult_t res;
    if (level_ < stateMachine->getLevel())
        return res;
    if ((valid_ && closed_) == false)
        return res;
    auto evolution = current_state_->evolve(stateMachine);
    if (evolution.first == nullptr)
        return res;
    if (current_state_->getId() == id_initial_state_ || age_ > stateMachine->getAge())
        age_ = stateMachine->getAge();
    last_update_ = stateMachine->getLastupdate();

    for (auto id_fact: stateMachine->getIdsFacts())
        id_facts_involve.push_back(id_fact);
    if (stateMachine->getCompletionRatio() != 1.0)
        incompletes_state_machines_.emplace_back(stateMachine, evolution.second->getRemap());

    current_state_ = evolution.first;
    new_explanations_ = checkIncompletsStateMachines();
    res.update_available = new_explanations_;
    res.state = FeedResult::EVOLVE;
    if (current_state_->isFinalNode())
        res.state = FeedResult::COMPLETE;
//    notify();
    return res;
}

bool StateMachine::evolve(Action* action)
{
    if ((valid_ && closed_) == false)
        return false;
    auto evolution = current_state_->evolve(action);

    if (evolution == nullptr)
        return false;
//    if (current_state_->getId() == id_initial_state_)
//        age_ = fact->getTimeStamp();
//    last_update_ = fact->getTimeStamp();
//
//    current_state_ = evolution;
//    id_facts_involve.push_back(fact->getId()); // prepare to id on facts.
//    new_explanations_ = checkIncompletsStateMachines();
    return true;
}

bool StateMachine::evolve(Task* task)
{
    if ((valid_ && closed_) == false)
        return false;
    auto evolution = current_state_->evolve(task);

    if (evolution == nullptr)
        return false;
//    if (current_state_->getId() == id_initial_state_)
//        age_ = fact->getTimeStamp();
//    last_update_ = fact->getTimeStamp();
//
//    current_state_ = evolution;
//    id_facts_involve.push_back(fact->getId()); // prepare to id on facts.
//    new_explanations_ = checkIncompletsStateMachines();
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

bool StateMachine::addTransition(const PatternTransitionStateMachine_t& transitionStateMachine)
{
    if (closed_ == false)
    {
        addState(transitionStateMachine.origin_);
        addState(transitionStateMachine.next_);
        for (auto& var: transitionStateMachine.remap_var_)
            insertVariable(var.second);

        auto type = StateMachine::types_table.get(transitionStateMachine.type_);
        TransitionAction transition(type, transitionStateMachine.next_, transitionStateMachine.remap_var_);
        states_[transitionStateMachine.origin_]->addTransition(transition, states_[transitionStateMachine.next_]);
        return true;
    } else
        return false;
}

bool StateMachine::addTransition(const HTNTransition_t& transition)
{
    int origin_id = transition.step * 10;
    int final_id = (transition.step + 1) * 10;
    addState(origin_id);
//    std::cout << "create state : " << std::to_string(origin_id) << std::endl;

    addState(final_id);
//    std::cout << "create state : " << std::to_string(final_id) << std::endl;

    for (auto& var: transition.arguments_)
        insertVariable(var.first);


    linkHTNTransition(origin_id, final_id, transition);
    int new_state = 1;
    for (const auto parent: states_[origin_id]->getParents_())
    {
        if (parent->validateConstraints(transition.id_contraints_order))
        {
//            std::cout << "parent : " << parent->toString()<< std::endl;
//            linkHTNTransition(parent->getId(), origin_id + new_state, transition);
            addState((transition.step * 10) + new_state);
            linkHTNTransition(parent->getId(), (transition.step * 10) + new_state, transition);
            states_[(transition.step * 10) + new_state]->closeTo(states_[(transition.step + 1) * 10], parent,
                                                                 states_[transition.step * 10]);
            new_state++;
        }
    }
    return true;
}

void StateMachine::linkHTNTransition(int initial_state, int final_state, const HTNTransition_t& transition)
{
    if (transition.type == TransitionType::Action)
    {
        TransitionAction t(transition.id_subtask, final_state, {}, transition.arguments_);
        states_[initial_state]->addTransition(t, states_[final_state]);
    }
    if (transition.type == TransitionType::ActionMethod)
    {
        TransitionActionMethod t(transition.id_subtask, final_state, transition.arguments_);
        states_[initial_state]->addTransition(t, states_[final_state]);
    }
    if (transition.type == TransitionType::Task)
    {
        TransitionTask t(transition.id_subtask, final_state, transition.arguments_);
        states_[initial_state]->addTransition(t, states_[final_state]);
    }
//    std::cout << "Add parents : " << std::to_string(initial_state) << std::endl;
    states_[final_state]->addParents(states_[initial_state]);
    states_[final_state]->addValidateConstraints(transition.id_contraints_order);
    states_[final_state]->addValidateConstraints(states_[initial_state]->getConstrains_());

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

    auto N = new StateMachine(type_str_, new_id);
    N->variables_ = variables_;
    N->variables_.at("self").literal = N->getName();
    N->descriptions_ = descriptions_;
    N->id_initial_state_ = id_initial_state_;
    N->id_final_state_ = id_final_state_;
    for (const auto& obs: list_observer_)
        N->attach(obs);

    for (auto& state: states_)
        N->addState(state.first);

    for (auto& state: N->states_)
    {
        for (auto& pair_transition: states_.at(state.first)->getNextFacts())
        {
            TransitionFact t = pair_transition.first;
            state.second->addTransition(t, N->states_.at(pair_transition.second->getId()));
        }
        for (auto& pair_transition: states_.at(state.first)->getNextStateMachines())
        {
            TransitionAction t = pair_transition.first;
            state.second->addTransition(t, N->states_.at(pair_transition.second->getId()));
        }
        for (auto& pair_transition: states_.at(state.first)->getNextActions())
        {
            TransitionActionMethod t = pair_transition.first;
            state.second->addTransition(t, N->states_.at(pair_transition.second->getId()));
        }
        for (auto& pair_transition: states_.at(state.first)->getNextTasks())
        {
            TransitionTask t = pair_transition.first;
            state.second->addTransition(t, N->states_.at(pair_transition.second->getId()));
        }
    }
    if (last_state_required != -1)
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
    states_.at(last_state_required)->addTimeoutTransition(states_.at(id_final_state_));
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
//    std::cout << "link " << this->getName() << std::endl;
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
        for (auto& nexts_state: pair_states.second->getNextFacts())
            id_states_nexts.insert(nexts_state.second->getId());
        for (auto& nexts_state: pair_states.second->getNextStateMachines())
            id_states_nexts.insert(nexts_state.second->getId());
        for (auto& nexts_state: pair_states.second->getNextActions())
            id_states_nexts.insert(nexts_state.second->getId());
        for (auto& nexts_state: pair_states.second->getNextTasks())
            id_states_nexts.insert(nexts_state.second->getId());
    }

    std::unordered_set<int> result;
    std::unordered_set<int> list_final_state;
    for (auto& state: states_)
    {
        if (id_states_nexts.find(state.first) == id_states_nexts.end())
            result.insert(state.first);
        if (state.second->isFinalNode())
            list_final_state.insert(state.first);
    }

    int nb_initial_state = result.size();
    if (result.size() == 0)
    {
        throw NoInitialStateStateMachineException();
    } else if (nb_initial_state > 1)
    {
        std::unordered_set<State*> invalid_states;
        for (auto res: result)
            invalid_states.insert(states_.at(res));

        throw MultiInitialStateStateMachineException(invalid_states);
    }
    if (list_final_state.size() == 0)
    {
        throw NoFinalStateStateMachineException();
    } else if (list_final_state.size() > 1)
    {
        std::unordered_set<State*> invalid_states;
        for (auto res: list_final_state)
            invalid_states.insert(states_.at(res));

        throw MultiFinalStateStateMachineException(invalid_states);
    }


    id_initial_state_ = *result.begin();
    id_final_state_ = *list_final_state.begin();
    current_state_ = states_.at(id_initial_state_);


}

bool StateMachine::updateVar(const std::map<std::string, std::string>& remap,
                             const std::map<std::string, Variable_t>& new_var)
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
    for (const auto& state: states_)
        state.second->set_new_id(new_id);
}

std::string StateMachine::getStrStructure()
{
    std::string res;
    for (auto& state: states_)
    {
        if (!res.empty())
            res += "\n\n";
        res += "id : " + std::to_string(state.first) + " " + state.second->toString();
    }
    return res;
}

void StateMachine::expandProperties(onto::OntologyManipulator* onto_manipulator)
{
    for (auto& state: states_)
        (state.second)->expandTransitions(onto_manipulator);
}
void StateMachine::notify()
{
    MessageType t;
    if (isFinished())
    {
        t = MessageType::Finished;
        if (getCompletionRatio() == 1)
            t = MessageType::Complete;
    }
    if (!updated_sub_state_machines_.empty())
        t = MessageType::Update;
    if (t != MessageType::None)
    {
//        std::cout << "notify state machine from " << full_name_<< " " << std::to_string(static_cast<int>(t)) << std::endl;
        for (const auto& obs: list_observer_)
            obs->updateStateMachine(t, this);
    }


}
bool StateMachine::timeEvolution(TimeStamp_t stamp, double time_to_live) //TODO check again
{
    if (stamp - getAge() > time_to_live)
    {
        if (current_state_->hasTimeoutTransition())
        {
            current_state_ = current_state_->doTimeoutTransition();
            notify();
            return true;
        } else
            return false;
    }
    return true;
}

} // namespace procedural