
#include <iostream>
#include <algorithm>
#include "procedural/builder/HTNBuilder.h"
#include "procedural/core/Types/Task.h"
#include <stdexcept>

namespace procedural {

void HTNBuilder::buildAction(const std::vector<PrimitiveActionParsed_t>& actions)
{

}


bool HTNBuilder::buildTask(std::vector<Abstract_task_t>& abstract_tasks,
                           const std::vector<PrimitiveActionParsed_t>& actions_htn,
                           const std::vector<Action*>& actions_, const std::string& path)
{
    if (not checkActions(actions_htn, actions_))
    {
        std::cout << "impossible to continue error on check action " << std::endl;
        throw std::bad_exception();
        return false;
    }

    Task* new_task;
    for (auto& abstract_task: abstract_tasks)
    {
        std::cout << abstract_task << std::endl;
        new_task = new Task(abstract_task.name);
        std::map<std::string, std::string> arguments;
        std::cout<< "Args of : " << abstract_task.name << std::endl;
        for (auto& arg: abstract_task.arguments)
        {
            arguments.emplace(arg.varname, arg.type);
            std::cout << "\t - "  << arg.varname << ": " << arg.type << std::endl;
        }




        new_task->addArguments(arguments);
        int compt = 0;
        int step = 0;
        for (auto& method: abstract_task.methods_)
        {
            auto new_method = new Method(abstract_task.name, compt);
            std::map<std::string, std::string> remap;
            for (auto& select: method.subtask.selections)
                remap.emplace(select.attribut, select.type);
            int compt_actions = 0;
            method.subtask.linkActions();
            auto ordered_actions(method.subtask.actions_);
            std::sort(ordered_actions.begin(), ordered_actions.end(),
                      [](const Ordered_Action_t& a1, const Ordered_Action_t& a2) {
                          return a1.after_id.size() < a2.after_id.size();
                      });
            for (const auto& action: ordered_actions)
            {
                auto t = createTransition(action, compt_actions, arguments, remap);
//                std::cout << "transition " << t.toString() << std::endl;
                new_method->addTransition(t);
                compt_actions++;

                new_method->saveDot(compt_actions, "", true);

            }
            new_task->addMethods(new_method);
            new_method->saveDot(step++, "", false, path,arguments);

            compt++;
        }
        tasks_.push_back(new_task);
    }
    return true;

}
bool HTNBuilder::checkActions(const std::vector<PrimitiveActionParsed_t>& actions_htn,
                              const std::vector<Action*>& action_)
{
    for (const auto& action: actions_htn)
    {
        bool find = false;
        for (const auto& actionType: action_) //TODO Optimiser avec while et find condition
        {
            if (action.name == actionType->getName())
            {
                find = true;
                actions_possible.insert(std::make_pair(action.name, TransitionType::Action));
            }
        }
        if (find == false)
        {
            std::cout << action.name << " not found on already process action" << std::endl;
            return false;
        }

    }
//    std::cout << "list of possible action : " << std::endl;
//    for (const auto& action: actions_possible)
//        std::cout << action.first << (action.second == TransitionType::Action ? " => Action" : " => ActionMethod")
//                  << std::endl;
    return true;
}
void HTNBuilder::displayTask()
{
    for (const auto& task: tasks_)
    {
        std::cout << task->getName() << ":" << std::endl;
        for (auto& method: task->getMethods())
            std::cout << method->getStrStructure() << "\n" << std::endl;
    }

}
HTNTransition_t
HTNBuilder::createTransition(const Ordered_Action_t& action, int compt_action,
                             const std::map<std::string, std::string>& arguments,
                             const std::map<std::string, std::string>& remap)
{
    HTNTransition_t t;
    t.id_contraints_order = action.after_id;
    t.id_subtask_parsed = action.id;
    t.step = compt_action;


    auto val = std::find_if(actions_possible.begin(), actions_possible.end(),
                            [action](const std::pair<std::string, TransitionType>& action_pair) {
                                return action_pair.first == action.name;
                            });
//                std::cout << "action find : " << val->first << std::endl;
    if (val != actions_possible.end())
    {
        if (val->second == TransitionType::Action)
        {
            t.id_subtask = Action::action_types.get(action.name);
            t.type = val->second;
        }

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
    return t;
}
} // procedural