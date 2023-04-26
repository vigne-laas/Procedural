#include <iostream>
#include <set>
#include "procedural/core/Graph/Network.h"
#include "procedural/core/Graph/TransitionNetwork.h"

namespace procedural {

WordTable Network::types_table;

Network::Network(const std::string& name, int id, uint32_t level) : type_str_(name),
                                                                    id_(id),
                                                                    closed_(false),
                                                                    valid_(false),
                                                                    age_(0, 0),
                                                                    last_update_(0, 0),
                                                                    level_(level), new_explanations_(false)
{
    full_name_ = type_str_ + " " + std::to_string(id);
    variables_.emplace("self", full_name_);
    type_ = Network::types_table.get(type_str_);
}

bool Network::evolve(Fact* fact)
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
    new_explanations_ = checkIncompletsNetworks();
    return true;
}

bool Network::evolve(Network* net)
{
    if (level_ < net->getLevel())
        return false;
    if ((valid_ && closed_) == false)
        return false;
    auto evolution = current_state_->evolve(net);
    if (evolution.first == nullptr)
        return false;
    if (current_state_->getId() == id_initial_state_)
        age_ = net->getLastupdate();
    last_update_ = net->getLastupdate();

    for (auto id_fact: net->getIdsFacts())
        id_facts_involve.push_back(id_fact);
    if (net->getCompletionRatio() != 1.0)
    {
        incompletes_networks_.emplace_back(net, evolution.second->getRemap());
//        std::cout << "incomplete add : " << net->getName() << std::endl;
    }
    current_state_ = evolution.first;
    new_explanations_ = checkIncompletsNetworks();
//    std::cout << "new_explanations_  : " << new_explanations_ <<std::endl;
    return true;
}


float Network::getCompletionRatio() const
{
    float incomplete = 0;
    for (const auto& var: variables_)
        if (var.second.getValue() == 0 && var.first != "self")
            incomplete++;
    return 1 - (incomplete / float(variables_.size()));
}


std::vector<std::string> Network::getDescription()
{
    std::vector<std::string> res;
//    res.resize(descriptions_.size());
    for (auto& description: descriptions_)
        res.push_back(description.explainExplicit());
    return res;
}

std::vector<std::string> Network::getLiteralVariables()
{
    std::vector<std::string> res;
    for (auto const& map_elemet: variables_)
        res.push_back(map_elemet.first);
    return res;
}


bool Network::addTransition(const PatternTransitionFact_t& pattern)
{
    if (closed_ == false)
    {
        addState(pattern.origin_state);
        addState(pattern.next_state);

        insertVariable(pattern.fact->getVarSubject());
        insertVariable(pattern.fact->getVarObject());

        TransitionFact t = TransitionFact(*(pattern.fact));
        states_[pattern.origin_state]->addTransition(t, states_[pattern.next_state]);
        return true;
    } else
        return false;

}

bool Network::addNetwork(const PatternTransitionNetwork_t& network)
{
    if (closed_ == false)
    {
        addState(network.origin_);
        addState(network.next_);
        for (auto& var: network.remap_var_)
            insertVariable(var.second);

        auto type = Network::types_table.get(network.type_);
        TransitionNetwork transition(type, network.remap_var_);
        states_[network.origin_]->addTransition(transition, states_[network.next_]);
        return true;
    } else
        return false;
}

bool Network::addDescription(const ActionDescription_t& des)
{
    descriptions_.emplace_back(des, variables_);
    return true;
}

bool Network::closeNetwork()
{
    linkNetwork();
    closed_ = true;
    processInitialState();
    valid_ = true;
    return valid_;
}

std::string Network::toString()
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

Network* Network::clone(int new_id, int last_state_required)
{
    if ((valid_ && closed_) == false)
        return nullptr;

    Network* N = new Network(type_str_, new_id);
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
        for (auto& pair_transition: states_.at(state.first)->getNextsNetworks())
        {
            TransitionNetwork t = pair_transition.first;
            state.second->addTransition(t, N->states_.at(pair_transition.second->getId()));
        }
    }
    N->addTimeoutTransition(last_state_required);
    N->closeNetwork();
    return N;
}

void Network::displayVariables()
{
    for (auto& var: variables_)
        std::cout << "key : " << var.first << " => " << var.second.toString() << std::endl;
}

std::string Network::describe(bool expl)
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

    if (incompletes_networks_.empty())
        return msg;

    msg += "\t // incomplete Network link : ";
    for (const auto& incomplete_net: incompletes_networks_)
    {
        msg += incomplete_net.network_->getName();
        msg += "\n\t new level : " + std::to_string(incomplete_net.network_->getCompletionRatio()) + "\n";
//         net.first->displayVariables();
    }


    return msg;
}

bool Network::involveFacts(const std::set<uint32_t>& facts)
{
    for (auto id_fact: id_facts_involve)
        if (facts.find(id_fact) == facts.end())
            return false;
    return true;
}

void Network::addTimeoutTransition(int last_state_required)
{
    states_.at(last_state_required)->addTimeoutTransition();
}


/* ------------------------------ private part ------------------------------ */

bool Network::checkIncompletsNetworks()
{
    updated_sub_networks_.clear();
    if (incompletes_networks_.empty())
        return false;
    for (auto& incomplete_net: incompletes_networks_)
        if (incomplete_net.network_->updateVar(incomplete_net.remap_variables_, variables_))
            updated_sub_networks_.push_back(incomplete_net.network_);

    return updated_sub_networks_.empty() == false;
}


void Network::addState(int id_state)
{
    if (states_.find(id_state) == states_.end())
        states_.emplace(id_state, new State(getName(), id_state));
}

void Network::linkNetwork()
{
    for (auto& state: states_)
    {
        state.second->linkVariables(variables_);
        //state.expandTransitions();
    }
    for (auto& des: descriptions_)
        des.linkVariables(variables_);
}

void Network::insertVariable(const std::string& variable)
{
    variables_.emplace(variable, variable);
}


void Network::processInitialState()
{
    std::unordered_set<uint32_t> id_states_nexts;
    for (auto& pair_states: states_)
    {
        for (auto& nexts_state: pair_states.second->getNextsFacts())
            id_states_nexts.insert(nexts_state.second->getId());
        for (auto& nexts_state: pair_states.second->getNextsNetworks())
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
        throw NoInitialStateNetworkException();
    } else if (nb_initial_state > 1)
    {
        // Maybe raise an error rather than printing text
        std::unordered_set<State*> invalid_states;
        for (auto res: result)
        {
            invalid_states.insert(states_.at(res));
        }
        throw MultiInitialStateNetworkException(invalid_states);
    } else
    {
        id_initial_state_ = *result.begin();
        current_state_ = states_.at(id_initial_state_);
    }
}
bool
Network::updateVar(const std::map<std::string, std::string>& remap, const std::map<std::string, Variable_t>& new_var)
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
void Network::displayTypesTable()
{
    for (int index = 0; index < Network::types_table.size(); index++)
        std::cout << " types_table[" << index << "] : " << types_table[index] << std::endl;
}


} // namespace procedural