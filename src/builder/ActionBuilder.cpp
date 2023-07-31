#include <iostream>

#include "procedural/builder/ActionBuilder.h"
#include "procedural/core/Types/ActionDescription.h"

namespace procedural {

ActionBuilder::ActionBuilder(const std::vector<ParsedSimpleAction_t>& simple_actions,
                             std::vector<ParsedComposedAction_t>& composed_actions)
{
    build(simple_actions, composed_actions, nullptr);
}

void ActionBuilder::build(const std::vector<ParsedSimpleAction_t>& simple_actions,
                          std::vector<ParsedComposedAction_t>& composed_actions,
                          onto::ObjectPropertyClient* client = nullptr)
{
    property_client_ = client;
    combineActions(simple_actions, composed_actions);
    buildSimpleAction(simple_actions);
    buildComposedAction(composed_actions);
    buildIncomplete();
}

void ActionBuilder::display()
{
    for (auto& action : actions_)
        std::cout << action->getName() << std::endl;
}

void ActionBuilder::buildSimpleAction(const std::vector<ParsedSimpleAction_t>& simple_actions)
{
    for (const auto& action: simple_actions)
    {
        std::vector<PatternTransitionFact_t> facts;
        std::vector<ActionDescription_t> descriptions;

        int last_required = 0;
        for (auto& fact : action.facts.facts_)
        {
            auto patternfact = new PatternFact(fact.insertion, fact.subject, fact.property, fact.object, fact.required);

            for (auto index = last_required; index <= fact.level; index++)
                facts.emplace_back(index, patternfact, fact.level + 1);

            if (fact.required)
                last_required = fact.level + 1;
        }

        for (auto& description : action.descriptions.descriptions)
            descriptions.emplace_back(description.subject, description.property, description.object);

        int ttl = (action.parameters.ttl == 0) ? 30 : action.parameters.ttl;
        Action new_spe_action(action.getName(), facts, {}, descriptions,
                              last_required,
                              property_client_,
                              ttl);
        auto action_to_add = std::find_if(actions_.begin(), actions_.end(),
                                          [action](ActionType* act) { return act->getName() == action.type; });
        (*action_to_add)->addActions(new_spe_action);
    }

}
void ActionBuilder::buildComposedAction(std::vector<ParsedComposedAction_t>& composed_actions)
{
    for (auto& action : composed_actions)
    {
        bool incomplete = false;
        std::vector<PatternTransitionFact_t> facts;
        std::vector<ActionDescription_t> descriptions;
        std::vector<PatternTransitionStateMachine_t> state_machines;

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
                        facts.emplace_back(index, patternfact, fact.level + 1);

                    if (fact.required)
                        last_required = fact.level + 1;

                }
            }

            for (auto& subnet: action.pattern.sub_state_machines)
            {
                if (subnet.level == current_level)
                {
                    if (completeRemap(subnet))
                    {
                        for (auto index_subnet = last_required; index_subnet <= subnet.level; index_subnet++)
                            state_machines.emplace_back(index_subnet, subnet.level + 1, subnet.getName(), subnet.remap);

                        if (subnet.required)
                            last_required = subnet.level + 1;
                    }
                    else
                        incomplete = true;
                }
            }

            current_level++;
        } while ((current_level != action.max_level) && (incomplete == false));

        if (incomplete)
            incomplete_creation_state_machine_.push_back(action);
        else
        {
            for (auto& description: action.descriptions.descriptions)
                descriptions.emplace_back(description.subject, description.property, description.object);

            int ttl = (action.parameters.ttl == 0) ? 30 : action.parameters.ttl;
            Action new_spe_action(action.getName(), facts, state_machines, descriptions,
                                  last_required,
                                  property_client_,
                                  ttl);
            auto action_to_add = std::find_if(actions_.begin(), actions_.end(),
                                              [action](ActionType* act) { return act->getName() == action.type; });
            (*action_to_add)->addActions(new_spe_action);
        }
    }
}

bool ActionBuilder::completeRemap(SubStateMachine_t& sub_state_machine)
{
    std::vector<std::string> sub_net_variables;
    auto test = [sub_state_machine](const ActionType* action) { return action->getName() == sub_state_machine.type; };
    auto result = std::find_if(actions_.begin(), actions_.end(), test);
    if (result != actions_.end())
    {
        auto specialized_action = (*result)->getActions();
        auto test_specialized = [sub_state_machine](const Action& action) {
            return action.getName() == sub_state_machine.getName();
        };
        auto result_specialized = std::find_if(specialized_action.begin(), specialized_action.end(), test_specialized);
        if (result_specialized != specialized_action.end())
            sub_net_variables = result_specialized->getLiteralVariables();
    }
    else
        return false;


    if (sub_net_variables.empty())
        return false;
    else
    {
        for (auto const& literal : sub_net_variables)
            if (sub_state_machine.remap.find(literal) == sub_state_machine.remap.end())
                sub_state_machine.remap.insert(std::make_pair(literal, literal));
        return true;
    }

}

void ActionBuilder::combineActions(const std::vector<ParsedSimpleAction_t>& simple_actions,
                                   const std::vector<ParsedComposedAction_t>& composed_actions)
{
    for (auto& action : simple_actions)
    {
        auto result = std::find_if(actions_.begin(), actions_.end(),
                                   [action](ActionType* act) { return act->getName() == action.type; });
        if (result == actions_.end())
            actions_.push_back(new ActionType(action.type));
    }

    for (auto& action : composed_actions)
    {
        auto result = std::find_if(actions_.begin(), actions_.end(),
                                   [action](ActionType* act) { return act->getName() == action.type; });
        if (result == actions_.end())
            actions_.push_back(new ActionType(action.type));
    }
}

void ActionBuilder::buildIncomplete()
{
    bool possible = true;
    for (auto& net : incomplete_creation_state_machine_)
    {
        for (auto& subnet : net.pattern.sub_state_machines)
        {
            auto test = [subnet](const ActionType* action) { return action->getName() == subnet.type; };
            auto result = std::find_if(actions_.begin(), actions_.end(), test);
            if (result != actions_.end())
            {
                auto specialized_action = (*result)->getActions();
                auto test_specialized = [subnet](const Action& action) {
                    return action.getName() == subnet.getName();
                };
                auto result_specialized = std::find_if(specialized_action.begin(), specialized_action.end(),
                                                       test_specialized);
                if (result_specialized == specialized_action.end())
                {
                    possible = false;
                    break;
                }
            }
            else
            {
                possible = false;
                break;
            }
        }
    }

    if (possible == false)
        std::cout << "impossible to create all state machines because at least one sub state machine is missing " << std::endl;
    else
        buildComposedAction(incomplete_creation_state_machine_);
}

} // namespace procedural