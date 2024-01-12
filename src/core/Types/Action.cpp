#include "procedural/core/Types/Action.h"
#include "procedural/core/Types/ActionMethod.h"

#include <cmath>
#include <iostream>
#include <set>
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>

namespace procedural {

WordTable Action::action_types;

Action::Action(const std::string& name,
               const std::vector<procedural::PatternTransitionFact_t>& patterns,
               const std::vector<PatternTransitionStateMachine_t>& transition_state_machines,
               const std::vector<ActionDescription_t>& descriptions,
               int last_state_required,
               onto::OntologyManipulator* onto_manipulator,
               double ttl) : name_(name),
                             is_valid_(false),
                             time_to_live_(ttl),
                             id_(0),
                             last_state_required_(last_state_required)
{

    state_machine_factory_ = new StateMachine(name_, id_);
    for (auto& pattern: patterns)
        state_machine_factory_->addTransition(pattern);
    for (auto& net_pattern: transition_state_machines)
        state_machine_factory_->addTransition(net_pattern);
    for (auto& description: descriptions)
        state_machine_factory_->addDescription(description);

    is_valid_ = state_machine_factory_->closeStateMachine();
    if (onto_manipulator != nullptr)
        state_machine_factory_->expandProperties(onto_manipulator);
    state_machine_factory_->addTimeoutTransition(last_state_required_);
    type_ = action_types.get(name_);
}

int Action::getNextId()
{
    return id_++;
}

std::set<uint32_t> Action::checkStateMachine(TimeStamp_t current_timestamp)
{
    std::unordered_set<StateMachine*> state_machines_to_del;

    std::set<uint32_t> set_valid_facts;
    for (auto state_machine: state_machines_)
    {
        if (state_machine->timeEvolution(current_timestamp, time_to_live_))
            finished_state_machines_.insert(state_machine);
        else
        {
            state_machines_to_del.insert(state_machine);
//            std::cout << "Del state machine due to age " << state_machine->getName() << "value : "
//                      << current_timestamp - state_machine->getAge() << "ttl : " << time_to_live_ << std::endl;
        }
    }

    for (auto& complete_state_machine: finished_state_machines_)
    {
//        std::cout << "state machine finish :" << complete_state_machine->getName() << std::endl;
//        std::cout << "explanation : " << complete_state_machine->describe(true) << std::endl;
//        std::cout << "facts involved : ";

        for (auto& id: complete_state_machine->getIdsFacts())
            set_valid_facts.insert(id);
        state_machines_.erase(complete_state_machine);
    }

    for (auto& state_machine: state_machines_)
    {
        if (state_machine->involveFacts(set_valid_facts))
        {
//            std::cout << "may delete this network involved :" << state_machine->getName() << std::endl;
            state_machines_to_del.insert(state_machine);
        }
    }

    for (auto state_machine_to_del: state_machines_to_del)
    {
//        std::cout << "deleted network  : " << state_machine_to_del->getName() << std::endl;
        state_machines_.erase(state_machine_to_del);
        delete state_machine_to_del;
    }

    return set_valid_facts;
}

void Action::clean()
{
    for (auto complete_state_machine: finished_state_machines_)
        if (complete_state_machine->getCompletionRatio() == 1.0)
            if (complete_state_machine->getStatus() == StateMachineStatus::Readed)
            {
//                std::cout << "Clean Finished SM : " << complete_state_machine->getName() << std::endl;
                finished_state_machines_.erase(complete_state_machine);
//                delete complete_state_machine;
            }


//    for (auto network_to_del : networks_to_del_)
//         delete network_to_del;

//    finished_state_machines_.clear();
//    networks_to_del_.clear();
}

void Action::cleanInvolve(const std::set<uint32_t>& list_valid_facts)
{
    std::vector<StateMachine*> state_machine_to_deletes;
    if (evolve_sub_action_)
    {
        evolve_sub_action_ = false;
        return;
    }

    for (auto& state_machine: state_machines_)
    {
        if (state_machine->involveFacts(list_valid_facts))
        {
//            std::cout << "may delete this network due to involve facts" << std::endl;
            state_machine_to_deletes.push_back(state_machine);
        }

    }

    for (auto state_machine: state_machine_to_deletes)
    {
//        std::cout << "delete : " << state_machine->getName() << std::endl;
        state_machines_.erase(state_machine);
        delete state_machine;
    }
}

EvolveResult_t Action::feed(Fact* fact, TimeStamp_t current_timestamp)
{
    EvolveResult_t res;
    updated_states_machines.clear();
    bool evolve = false;
    if ((current_timestamp - fact->getTimeStamp()) > this->getTtl())
    {
        std::cout << "Reject on timeStamp ttl" << std::endl;
        return res;
    }


    for (auto& state_machine: state_machines_)
    {
        auto result = state_machine->evolve(fact);
        switch (result.state)
        {

            case FeedResult::NO_EVOLUTION:
                break;
            case FeedResult::EVOLVE:
                std::cout << "\t succes of evolution  : " << state_machine->getName() << std::endl;
                evolve = true;
                if (res.state != FeedResult::FINISH)
                    res.state = FeedResult::EVOLVE;
                break;
            case FeedResult::FINISH:
                std::cout << "\t succes of evolution  : " << state_machine->getName() << "reach final state"
                          << std::endl;
                evolve = true;
                finished_state_machines_.insert(state_machine);
                res.state = FeedResult::FINISH;
                break;
        }
        if (result.update_available)
        {
            updated_states_machines.push_back(state_machine);
            res.update_available = true;
        }


    }

    if (evolve == false)
    {
        StateMachine* new_net = state_machine_factory_->clone(-1, last_state_required_);
        auto result = new_net->evolve(fact);
        switch (result.state)
        {
            case FeedResult::NO_EVOLUTION:
                delete new_net;
                break;
            case FeedResult::EVOLVE:
                new_net->setId(getNextId());
//                new_net->attach(this);
                state_machines_.insert(new_net);
                std::cout << "create new state machine " << new_net->getName() << std::endl;
                if (res.state != FeedResult::FINISH)
                    res.state = FeedResult::EVOLVE;
                break;
            case FeedResult::FINISH:
                new_net->setId(getNextId());
                finished_state_machines_.insert(new_net);
                break;
        }
//        if (result.state == FeedResult::EVOLVE)
//        {
//            new_net->setId(getNextId());
//            new_net->attach(this);
//            state_machines_.insert(new_net);
////            std::cout << "create new state machine " << new_net->getName() << std::endl;
//            evolve = true;
//            res.insert(FeedResult::EVOLVE);
//        } else
//            delete new_net;
    }
    return res;
}

EvolveResult_t Action::checksubAction(Action* action)
{

    EvolveResult_t res;
    updated_states_machines.clear();
    std::unordered_set<StateMachine*> complete_state_machines = action->getFinishedStateMachine();

    if (complete_state_machines.empty())
        return res;

    bool evolve = false;
    for (auto& state_machine: state_machines_)
    {
        for (auto complete_state_machine: complete_state_machines)
        {
            auto result = state_machine->evolve(complete_state_machine);
            switch (result.state)
            {

                case FeedResult::NO_EVOLUTION:
                    break;
                case FeedResult::EVOLVE:
                    std::cout << "\t succes of evolution sub action : " << state_machine->getName() << std::endl;
                    evolve = true;
                    evolve_sub_action_ = true;
                    if (res.state != FeedResult::FINISH)
                        res.state = FeedResult::EVOLVE;
                    break;
                case FeedResult::FINISH:
                    res.state = FeedResult::FINISH;
                    break;
            }
            if (result.update_available)
            {
                for (auto& net: state_machine->getUpdatedStateMachines())
                    updated_states_machines.push_back(net);
            }
        }

    }
    if (evolve == false)
    {
        for (auto complete_state_machine: complete_state_machines)
        {
            StateMachine* new_net = state_machine_factory_->clone(-1, last_state_required_);
            auto result = new_net->evolve(complete_state_machine);
            switch (result.state)
            {
                case FeedResult::NO_EVOLUTION:
                    delete new_net;
                    break;
                case FeedResult::EVOLVE:
                    new_net->setId(getNextId());
//                    new_net->attach(this);
                    state_machines_.insert(new_net);
                    std::cout << "create new state machine " << new_net->getName() << std::endl;
                    if (res.state != FeedResult::FINISH)
                        res.state = FeedResult::EVOLVE;
                    break;
                case FeedResult::FINISH:
                    new_net->setId(getNextId());
                    finished_state_machines_.insert(new_net);
                    break;
            }
//            {
//                new_net->setId(getNextId());
//                new_net->attach(this);
//                state_machines_.insert(new_net);
//                evolve_sub_action_ = true;
//                evolve = true;
//                res.insert(FeedResult::EVOLVE);
//            } else
//            delete new_net;
        }
    }

    return res;
}

std::string Action::currentState(bool short_version)
{
    std::string res;
    res += "Pattern Recognition of : " + name_ + "\t";
    res += "\t nb of state machine : " + std::to_string(state_machines_.size()) + "\n";
    for (auto& net: state_machines_)
    {
        if (short_version)
            res += net->getCurrentState()->toString() + "\t";
        else
            res += net->toString() + "\n";
    }

    res += "\n";
    return res;
}

std::string Action::toString()
{
    std::string res;
    res += "Pattern Recognition of : " + name_ + "\n";
    res += "\t nb of active state machine : " + std::to_string(state_machines_.size()) + "\n";
    res += "\t active state machine : \n";
    for (auto& net: state_machines_)
        res += net->toString() + "\n";
    return res;
}

std::string Action::getStrStructure()
{
    std::string res;
    res += "Structure of : " + name_ + "\n";
    res += state_machine_factory_->getStrStructure();
    return res;
}


std::vector<StateMachine*> Action::getNewExplanation()
{
    return updated_states_machines;
}


} // namespace procedural