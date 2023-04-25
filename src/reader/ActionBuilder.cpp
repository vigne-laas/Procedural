#include <iostream>
#include "procedural/reader/ActionBuilder.h"
#include "procedural/core/Types/ActionDescription.h"

namespace procedural {
ActionBuilder::ActionBuilder(const std::vector<ParsedSimpleAction_t>& simpleActions,
                             std::vector<ParsedComposedAction_t>& composedActions)
{
    buildSimpleAction(simpleActions);
    buildComposedAction(composedActions);
}
void ActionBuilder::buildSimpleAction(const std::vector<ParsedSimpleAction_t>& simple_actions)
{
    for(const auto& action: simple_actions)
    {

        std::vector<PatternTransitionFact_t> facts;
        std::vector<ActionDescription_t> descriptions;
        std::vector<PatternTransitionNetwork_t> networks;

        int last_required = 0;
        for(auto& fact : action.facts.facts_)
        {
            auto patternfact = new PatternFact(fact.insertion,fact.subject,fact.property,fact.object, fact.required);
            for (auto index = last_required; index <= fact.level; index++)
                facts.emplace_back(fact.level,patternfact,fact.level+1);
            if(fact.required)
                last_required = fact.level+1;
        }

        for(auto& description : action.descriptions.descriptions)
            descriptions.emplace_back(description.subject,description.property,description.object);

        auto new_action  = new SpecializedAction(action.name_,facts,networks,descriptions,last_required,action.parameters.ttl);
        actions_.push_back(new_action);

    }

}
void ActionBuilder::buildComposedAction(std::vector<ParsedComposedAction_t>& composed_actions)
{
    for(auto& action: composed_actions)
    {
        std::vector<PatternTransitionFact_t> facts;
        std::vector<ActionDescription_t> descriptions;
        std::vector<PatternTransitionNetwork_t> networks;

        int last_required = 0;
        for(auto& fact : action.pattern.facts)
        {
            auto patternfact = new PatternFact(fact.insertion,fact.subject,fact.property,fact.object, fact.required);
            for (auto index = last_required; index <= fact.level; index++)
                facts.emplace_back(index,patternfact,fact.level+1);
            if(fact.required)
                last_required = fact.level+1;
        }

        for(auto& subnet : action.pattern.subnetworks)
        {
            if(completeRemap(subnet))
            {
                for (auto index = last_required; index <= subnet.level; index++)
                    networks.emplace_back(index,subnet.level+1,subnet.type,subnet.remap);
                if(subnet.required)
                    last_required = subnet.level+1;
            }



        }

        for(auto& description : action.descriptions.descriptions)
            descriptions.emplace_back(description.subject,description.property,description.object);

        auto new_action  = new SpecializedAction(action.name_,facts,networks,descriptions,last_required,action.parameters.ttl);
        actions_.push_back(new_action);

    }

}

void ActionBuilder::display()
{
    std::cout << "size : " << actions_.size()<<std::endl;
}
bool ActionBuilder::completeRemap(SubNetwork_t& network)
{
    auto test = [network](const SpecializedAction* action){return action->getName()==network.type;};
    auto result = std::find_if(actions_.begin(),actions_.end(),test);
//    if(result != actions_.end())
//    {
//        (*result)->
//    }



    return true;
}
} // procedural