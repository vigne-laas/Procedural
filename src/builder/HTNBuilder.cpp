
#include <iostream>
#include <algorithm>
#include "procedural/builder/HTNBuilder.h"
#include "procedural/core/Types/Task.h"

namespace procedural {

void HTNBuilder::buildAction(const std::vector<PrimitiveActionParsed_t>& actions)
{

}
void HTNBuilder::buildTask(const std::vector<Abstract_task_t>& abstract_tasks,
                           const std::vector<PrimitiveActionParsed_t>& actions_htn,
                           const std::vector<ActionMethod*>& actions_type)
{
    if (not checkActions(actions_htn, actions_type))
    {
        std::cout << "impossible to continue error on check action " << std::endl;
        return;
    }

    Task* new_task;
    for (const auto& abstract_task: abstract_tasks)
    {
        std::cout << abstract_task << std::endl;
        new_task = new Task(abstract_task.name);
        std::map<std::string, std::string> arguments;
        for (const auto& arg: abstract_task.arguments)
            arguments.emplace(arg.varname, arg.type);
        new_task->addArguments(arguments);
        int compt = 0;
        for (const auto& method: abstract_task.methods_)
        {
            Method new_method(abstract_task.name, compt);
            std::map<std::string, std::string> remap;
            for (const auto& select: method.subtask.selections)
                remap.emplace(select.attribut, select.type);
            int compt_actions = 0;
            auto ordered_actions(method.subtask.actions_);
            std::sort(ordered_actions.begin(), ordered_actions.end(),
                      [](const Ordered_Action_t& a1, const Ordered_Action_t& a2) {
                          return a1.after_id.size() < a2.after_id.size();
                      });
            for (const auto& action: ordered_actions)
            {
                HTNTransition_t t;
                t.id_contraints_order = action.after_id;
                t.id_subtask_parsed = action.id;
                t.step = compt_actions;

                auto val = std::find_if(actions_possible.begin(), actions_possible.end(),
                                        [action](const std::string& action_) {
                                            return action_ == action.name;
                                        });
                if (val != actions_possible.end())
                {
                    t.id_subtask = Action::action_types.get(action.name);
                    t.type = TransitionType::Action;
                } else
                {
                    t.id_subtask = Task::task_types.get(action.name);
                    t.type = TransitionType::Task;
                }
                for (auto& arg: action.arguments)
                {
                    if (auto remap_val = remap.find(arg); remap_val != remap.end())
                        t.arguments_.emplace(*remap_val);
                    else if (auto remap_val_task = arguments.find(arg); remap_val_task != arguments.end())
                        t.arguments_.emplace(*remap_val_task);
                    else
                        std::cout << "Error arg non find in remap : " << arg << std::endl;
                }
                compt_actions++;
                new_method.addTransition(t);
            }
            new_task->addMethods(new_method);
            compt++;
        }
        tasks_.push_back(new_task);
    }

}
bool HTNBuilder::checkActions(const std::vector<PrimitiveActionParsed_t>& actions_htn,
                              const std::vector<ActionMethod*>& action_type)
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
                {
                    find = true;
                    actions_possible.emplace_back(val->getName());
                }

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
    for (const auto& task: tasks_)
    {
        std::cout << task->getName() << ":" << std::endl;
        for (auto& method: task->getMethods())
            std::cout << method.getStrStructure() << "\n" << std::endl;
    }

}
} // procedural