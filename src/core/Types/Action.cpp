#include "procedural/core/Types/Action.h"
#include "procedural/core/Types/ActionType.h"

#include <cmath>
#include <iostream>
#include <set>
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>

namespace procedural {


Action::Action(const std::string& name,
               const std::vector<procedural::PatternTransitionFact_t>& patterns,
               const std::vector<PatternTransitionNetwork_t>& patterns_network,
               const std::vector<ActionDescription_t>& descriptions,
               int last_state_required,
               ObjectPropertyClient* object_client,
               double ttl) : name_(name),
                                                   is_valid_(false),
                                                   time_to_live_(ttl),
                                                   id_(0),
                                                   last_state_required_(
                                                           last_state_required)
{

    state_machine_factory_ = new StateMachine(name_, id_);
    for (auto& pattern: patterns)
        state_machine_factory_->addTransition(pattern);
    for (auto& net_pattern: patterns_network)
        state_machine_factory_->addNetwork(net_pattern);
    for (auto& description: descriptions)
        state_machine_factory_->addDescription(description);

    is_valid_ = state_machine_factory_->closeNetwork();
    if (object_client != nullptr)
        state_machine_factory_->expandProperties(object_client);
    state_machine_factory_->addTimeoutTransition(last_state_required_);
}


int Action::getNextId()
{
    return id_++;
}

std::set<uint32_t> Action::checkNetwork(TimeStamp_t current_timestamp)
{
//    std::unordered_set<Network*> complete_networks;
    std::unordered_set<StateMachine*> networks_to_del;

    std::set<uint32_t> set_valid_facts;
    for (auto network: networks_)
    {
//        std::cout << "value : "<< current_timestamp - network->getAge() << "ttl : " << time_to_live_ << std::endl;
        if (network->isComplete())
            complete_networks_.insert(network);
        else if (current_timestamp - network->getAge() > time_to_live_)
        {

            if(network->getCurrentState()->hasTimeoutTransition())
                complete_networks_.insert(network);
            else
            {
                networks_to_del.insert(network);
                std::cout << "Del network due to age " << network->getName() << "value : "
                          << current_timestamp - network->getAge() << "ttl : " << time_to_live_ << std::endl;
            }
        }
    }

    for (auto& complete_network: complete_networks_)
    {
//        std::cout << "network finish :" << complete_network->getName() << std::endl;
//        std::cout << "explanation : " << complete_network->describe(true) << std::endl;
//        std::cout << "facts involved : ";

        for (auto& id: complete_network->getIdsFacts())
        {
//            std::cout << std::to_string(id) << "|";
            set_valid_facts.insert(id);
        }
//        NetworkOutput output(complete_network,false);
//        std::cout << output << std::endl;
//        std::cout << std::endl;
        networks_.erase(complete_network);
        // delete complete_network; // Check if Guillaume is right
    }
    // std::cout << "Pattern : "<< this->name_<< std::endl;
    // std::cout << "size net : "<< networks_.size()<< std::endl;
    for (auto& network: networks_)
    {
        if (network->involveFacts(set_valid_facts))
        {
//            std::cout << "may delete this network involved :" << network->getName() << std::endl;
            networks_to_del.insert(network);
        }
    }

    for (auto network_to_del: networks_to_del)
    {
//        std::cout << "deleted network  : " << network_to_del->getName() << std::endl;
        networks_.erase(network_to_del);
        delete network_to_del;
    }

    return set_valid_facts;
}

void Action::clean()
{
    for (auto complete_net: complete_networks_)
        if (complete_net->getCompletionRatio() == 1.0)
            delete complete_net;

//    for (auto network_to_del : networks_to_del_)
//         delete network_to_del;

    complete_networks_.clear();
//    networks_to_del_.clear();
}

void Action::cleanInvolve(const std::set<uint32_t>& list_valid_facts)
{
    std::vector<StateMachine*> network_to_deletes;
    if (evolve_sub_action)
    {
        evolve_sub_action = false;
        return;
    }

    for (auto& network: networks_)
    {
        if (network->involveFacts(list_valid_facts))
        {
//            std::cout << "may delete this network involved : " << network->getName() << std::endl;
            network_to_deletes.push_back(network);
        }
    }

    for (auto network: network_to_deletes)
    {
//        std::cout << "deleted network  : " << network->getName() << std::endl;
        networks_.erase(network);
        delete network;
    }
}

bool Action::feed(Fact* fact)
{
    updated_networks.clear();
    bool evolve = false;
    for (auto& network: networks_)
    {
        if (network->evolve(fact))
        {
            std::cout << "\t succes of evolution  : " << network->getName() << std::endl;
            evolve = true;
            if (network->newExplanationAvailable())
                updated_networks.push_back(network);
            // checkNetworkComplete(network);
        }
    }


    if (evolve == false)
    {
        StateMachine* new_net = state_machine_factory_->clone(-1, last_state_required_);
        if (new_net->evolve(fact))
        {
            new_net->setId(getNextId());
            networks_.insert(new_net);
            std::cout << "create new network " << new_net->getName() << std::endl;

            evolve = true;
            // checkNetworkComplete(new_net);
        } else
            delete new_net;
    }

    return evolve;

}
bool Action::checksubAction(ActionType* action)
{
    updated_networks.clear();
    std::unordered_set<StateMachine*> complete_networks = action->getCompleteNetworks();

    if (complete_networks.empty())
        return false;

    bool evolve = false;
    for (auto& network: networks_)
    {
        for (auto complete_net: complete_networks)
            if (network->evolve(complete_net))
            {
                std::cout << "\t succes of evolution sub action : " << network->getName() << std::endl;
                evolve = true;
                evolve_sub_action = true;
                if (network->newExplanationAvailable())
                    for (auto& net: network->getUpdatedNetworks())
                        updated_networks.push_back(net);
                // checkNetworkComplete(network);
            }

    }

    if (evolve == false)
    {
        for (auto complete_net: complete_networks)
        {
            StateMachine* new_net = state_machine_factory_->clone(-1, last_state_required_);
            if (new_net->evolve(complete_net))
            {
//                std::cout << "create new network " << new_net->getName() << std::endl;
                new_net->setId(getNextId());
                networks_.insert(new_net);
                evolve_sub_action = true;
                evolve = true;
                // checkNetworkComplete(new_net);
            } else
                delete new_net;
        }
    }
    return evolve;
//    std::cout << "size network : " << networks_.size() << std::endl;
}

std::string Action::toString()
{
    std::string res;
    res += "Pattern Recognition of : " + name_ + "\n";
    res += "\t nb of active networks : " + std::to_string(networks_.size()) + "\n";
    res += "\t active networks : \n";
    for (auto& net: networks_)
        res += net->toString() + "\n";
    return res;
}

std::string Action::currentState(bool shortVersion)
{
    std::string res;
    res += "Pattern Recognition of : " + name_ + "\t";
    res += "\t nb of networks : " + std::to_string(networks_.size()) + "\n";
    for (auto& net: networks_)
    {
        if (shortVersion)
            res += net->getCurrentState()->toString() + "\t";
        else
            res += net->toString() + "\n";
    }

    res += "\n";
    return res;
}
std::string Action::getStrStructure()
{
    std::string res;
    res += "Structure of : " + name_ + "\n";
    res += state_machine_factory_->getStrStructure();
    return res;
}


} // namespace procedural