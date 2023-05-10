#include <iostream>

#include "procedural/reader/ActionBuilder.h"
#include "procedural/core/Types/ActionDescription.h"

namespace procedural {
ActionBuilder::ActionBuilder(const std::vector<ParsedSimpleAction_t>& simple_actions,
                             std::vector<ParsedComposedAction_t>& composed_actions)
{
    build(simple_actions, composed_actions, nullptr);
}

void ActionBuilder::build(const std::vector<ParsedSimpleAction_t>& simple_actions,
                          std::vector<ParsedComposedAction_t>& composed_actions,
                          ObjectPropertyClient* client = nullptr)
{
    property_client_=client;
    combineActions(simple_actions, composed_actions);
    buildSimpleAction(simple_actions);
    buildComposedAction(composed_actions);
    buildIncomplete();
}


void ActionBuilder::buildSimpleAction(const std::vector<ParsedSimpleAction_t>& simple_actions)
{
    for (const auto& action: simple_actions)
    {

        std::vector<PatternTransitionFact_t> facts;
        std::vector<ActionDescription_t> descriptions;
        std::vector<PatternTransitionNetwork_t> networks;

        int last_required = 0;
//        std::cout << action.type << "_" << action.subtype << std::endl;
        for (auto& fact: action.facts.facts_)
        {
            auto patternfact = new PatternFact(fact.insertion, fact.subject, fact.property, fact.object, fact.required);
//            std::cout << " fact parsed : " << fact << " pattern : " << patternfact->toString() << std::endl;
            for (auto index = last_required; index <= fact.level; index++)
            {
//                std::cout << "link " << fact << " from " << index << " to " << fact.level + 1 << std::endl;
                facts.emplace_back(index, patternfact, fact.level + 1);
            }

            if (fact.required)
                last_required = fact.level + 1;
        }

        for (auto& description: action.descriptions.descriptions)
            descriptions.emplace_back(description.subject, description.property, description.object);
        int ttl = action.parameters.ttl;
        if (ttl == 0)
            ttl = 30;

        SpecializedAction new_spe_action(action.getName(), facts, networks, descriptions,
                                         last_required,
                                         property_client_,
                                         ttl);
        auto action_to_add = std::find_if(actions_.begin(), actions_.end(),
                                          [action](Action* act) { return act->getName() == action.type; });
        (*action_to_add)->addPatterns(new_spe_action);
    }

}
void ActionBuilder::buildComposedAction(std::vector<ParsedComposedAction_t>& composed_actions)
{
    for (auto& action: composed_actions)
    {
        bool incomplete = false;
        std::vector<PatternTransitionFact_t> facts;
        std::vector<ActionDescription_t> descriptions;
        std::vector<PatternTransitionNetwork_t> networks;

        int last_required = 0;
        int current_level = 0;
        do
        {
            for (auto& fact: action.pattern.facts)
            {
                if (fact.level == current_level)
                {
                    auto patternfact = new PatternFact(fact.insertion, fact.subject, fact.property, fact.object,
                                                       fact.required);
                    for (auto index = last_required; index <= fact.level; index++)
                    {
//                        std::cout << "link fact on " << action.getName() << " with " << fact << " between : " << index
//                                  << " and " << fact.level + 1 << std::endl;
                        facts.emplace_back(index, patternfact, fact.level + 1);
                    }

                    if (fact.required)
                        last_required = fact.level + 1;

                }
            }

            for (auto& subnet: action.pattern.subnetworks)
            {
                if (subnet.level == current_level)
                {
                    if (completeRemap(subnet))
                    {
                        for (auto index_subnet = last_required; index_subnet <= subnet.level; index_subnet++)
                        {
//                            std::cout << "link subnet on " << action.getName() << " with " << subnet
//                                      << " between : " << index_subnet << " and " << subnet.level + 1 << std::endl;
                            networks.emplace_back(index_subnet, subnet.level + 1, subnet.getName(), subnet.remap);
                        }

                        if (subnet.required)
                            last_required = subnet.level + 1;
                    } else
                        incomplete = true;

                }


            }
            current_level++;
        } while ((current_level != action.max_level) && (incomplete == false));
//        std::cout << "-----------------------------------------------------------------------------------------"
//                  << std::endl;
        if (incomplete)
        {
//            std::cout << "impossible to create action due to incomplete remap" << std::endl;
            incomplete_creation_network_.push_back(action);

        } else
        {
            for (auto& description: action.descriptions.descriptions)
                descriptions.emplace_back(description.subject, description.property, description.object);
//            for (auto net: networks)
//                std::cout << "net types  : " << net.type_ << std::endl;
            int ttl = action.parameters.ttl;
            if (ttl == 0)
                ttl = 30;
            SpecializedAction new_spe_action(action.getName(), facts, networks, descriptions,
                                             last_required,
                                             property_client_,
                                             ttl);
            auto action_to_add = std::find_if(actions_.begin(), actions_.end(),
                                              [action](Action* act) { return act->getName() == action.type; });
            (*action_to_add)->addPatterns(new_spe_action);
        }


    }

}

void ActionBuilder::display()
{
    for (auto action: actions_)
        std::cout << action->getName() << std::endl;
}
bool ActionBuilder::completeRemap(SubNetwork_t& network)
{
    std::vector<std::string> sub_net_variables;
    auto test = [network](const Action* action) { return action->getName() == network.type; };
    auto result = std::find_if(actions_.begin(), actions_.end(), test);
    if (result != actions_.end())
    {
        auto specialized_action = (*result)->getSpecializedActions();
        auto test_specialized = [network](const SpecializedAction& action) {
            return action.getName() == network.getName();
        };
        auto result_specialized = std::find_if(specialized_action.begin(), specialized_action.end(), test_specialized);
        if (result_specialized != specialized_action.end())
            sub_net_variables = result_specialized->getLiteralVariables();
    } else
        return false;


    if (sub_net_variables.empty())
        return false;
    else
    {
        for (auto const& literal: sub_net_variables)
            if (network.remap.find(literal) == network.remap.end())
                network.remap.insert(std::make_pair(literal, literal));
        return true;
    }

}
void ActionBuilder::combineActions(const std::vector<ParsedSimpleAction_t>& simple_actions,
                                   std::vector<ParsedComposedAction_t>& composed_actions)
{

    for (auto& action: simple_actions)
    {
        auto result = std::find_if(actions_.begin(), actions_.end(),
                                   [action](Action* act) { return act->getName() == action.type; });
        if (result == actions_.end())
        {
//            std::cout << "create new action :" << action.type << std::endl;
            auto action_ = new Action(action.type);
            actions_.push_back(action_);
        }

    }
    for (auto& action: composed_actions)
    {
        auto result = std::find_if(actions_.begin(), actions_.end(),
                                   [action](Action* act) { return act->getName() == action.type; });
        if (result == actions_.end())
        {
//            std::cout << "create new action :" << action.type << std::endl;
            auto action_ = new Action(action.type);
            actions_.push_back(action_);
        }

    }


}
void ActionBuilder::buildIncomplete()
{
    bool possible = true;
    for (auto& net: incomplete_creation_network_)
    {
        for (auto& subnet: net.pattern.subnetworks)
        {
            auto test = [subnet](const Action* action) { return action->getName() == subnet.type; };
            auto result = std::find_if(actions_.begin(), actions_.end(), test);
            if (result != actions_.end())
            {
                auto specialized_action = (*result)->getSpecializedActions();
                auto test_specialized = [subnet](const SpecializedAction& action) {
                    return action.getName() == subnet.getName();
                };
                auto result_specialized = std::find_if(specialized_action.begin(), specialized_action.end(),
                                                       test_specialized);
                if (result_specialized == specialized_action.end())
                    possible = false;


            } else
                possible = false;
        }
        if (possible == false)
            break;
    }
    if (possible == false)
        std::cout << "impossible to create all networks because at least one sub network is missing " << std::endl;
    else
        buildComposedAction(incomplete_creation_network_);

}

} // procedural