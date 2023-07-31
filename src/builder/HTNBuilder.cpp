
#include <iostream>
#include "procedural/builder/HTNBuilder.h"


namespace procedural {
void procedural::HTNBuilder::buildTask(const std::vector<MethodParsed_t>& methods)
{
    ActionType* actionType;
    for (const auto& method: methods)
    {
        std::cout << method << std::endl;
        actionType = new ActionType(method.name);
        for (const auto& decomp: method.decomposition_)
        {
            bool incomplete = false;
            std::vector<PatternTransitionFact_t> facts;
            std::vector<ActionDescription_t> descriptions;
            std::vector<PatternTransitionStateMachine_t> state_machines;

            int last_required = 0;
            int current_level = 0;
            for (const auto& subtask: decomp.subtask.actions_)
            {

                do
                {
                    for (auto index_subnet = last_required; index_subnet <= current_level; index_subnet++)
                    {
//                            std::cout << "link subnet on " << action.getName() << " with " << subnet
//                                      << " between : " << index_subnet << " and " << subnet.level + 1 << std::endl;
//                        state_machines.emplace_back(index_subnet, current_level + 1, subtask.name, {});
                    }

//                    if (subnet.required)
//                        last_required = subnet.level + 1;

                } while ((current_level != decomp.subtask.actions_.size()) && (incomplete == false));

            }
        }


    }


}
void HTNBuilder::buildAction(const std::vector<PrimitiveActionParsed_t>& actions)
{

}
void HTNBuilder::buildTask(const std::vector<MethodParsed_t>& methods,
                           const std::vector<PrimitiveActionParsed_t>& actions_htn,
                           const std::vector<ActionType*>& actions_type)
{
    if (not checkActions(actions_htn, actions_type))
    {
        std::cout << "impossible to continue eror on check action " << std::endl;
        return;
    }

    ActionType* actionType;
    for (const auto& task: methods)
    {
        std::cout <<"Create task : " <<task.name << std::endl;
        actionType = new ActionType(task.name);
        for (auto decomp = task.decomposition_.begin(); decomp < task.decomposition_.end(); decomp++)
        {


            bool incomplete = false;
            std::vector<PatternTransitionFact_t> facts;
            std::vector<ActionDescription_t> descriptions;
            std::vector<PatternTransitionStateMachine_t> state_machines;
            std::map<std::string, std::string> remap_var_;

            int last_required = 0;
            int current_level = 0;
            int limit_size = (int) decomp->subtask.actions_.size();
            for (const auto& subtask: decomp->subtask.actions_)
            {
                do
                {
                    for (auto index_subnet = last_required; index_subnet <= current_level; index_subnet++)
                    {
//                        std::cout << "link subnet on " << task.name << " with " << subtask.name
//                                  << " between : " << index_subnet << " and " << current_level + 1 << std::endl;
                        state_machines.emplace_back(index_subnet, current_level + 1, subtask.name, remap_var_);
                    }
                    current_level++;

//                    if (subnet.required)
//                        last_required = subnet.level + 1;

                } while ((current_level < limit_size) && (incomplete == false));

            }
            auto id = std::distance(task.decomposition_.begin(),decomp);

            Action action_(task.name + "_Method_"+std::to_string(id),facts,state_machines,descriptions,limit_size,
                                    nullptr);
            std::cout << "create new action : " << action_.toString() << std::endl;
            actionType->addActions(action_);
        }
        actionTypes.push_back(actionType);
    }

}
bool HTNBuilder::checkActions(const std::vector<PrimitiveActionParsed_t>& actions_htn,
                              const std::vector<ActionType*>& action_type)
{
    for (const auto& action: actions_htn)
    {
        bool find = false;
        for (const auto& actionType: action_type)
        {
            if (find == false)
            {
                auto actions_ = actionType->getActions();
                auto val = std::find_if(actions_.begin(), actions_.end(),
                                        [action](const Action& action_primitive) {
                                            return action_primitive.getName() == action.name;
                                        });
                if (val != actions_.end())
                    find = true;
            }
        }
        if (find == false)
        {
            std::cout << action.name << " not found on already process action" << std::endl;
            return false;
        }

    }
    return true;
}
void HTNBuilder::displayActions()
{
    for (const auto& actionType: actionTypes)
    {
        std::cout << actionType->getName() << ":" << std::endl;
        for (auto& action: actionType->getActions())
            std::cout << action.getStrStructure() << std::endl;
    }

}
} // procedural