#include <vector>
#include "procedural/core/TaskRecognition.h"
#include "procedural/core/Types/Task.h"
#include "procedural/core/Types/ActionMethod.h"
#include "procedural/core/Types/Action.h"
namespace procedural {

void TaskRecognition::init(const std::vector<Task*>& tasks, double ttl)
{
    tasks_ = tasks;
    callback_output_ = TaskRecognition::defaultCallback;
    for (const auto& task: tasks_)
    {
        task->attach(this);
    }


}
void TaskRecognition::process(TimeStamp_t current_time)
{
    for (auto task: tasks_)
    {
        for (const auto& f_action_method: finished_actions_method_)
            task->feed(f_action_method);

        for (const auto& f_action: finished_actions_)
            task->feed(f_action);

        int nb_update;
        do
        {
            nb_update = 0;
            std::vector<TaskRecognizedMSG_t> msg_finished_task_;
            std::vector<Task*> finished_task = finished_task_;

            for (const auto& f_task: finished_task_)
                if (task->feed(f_task))
                    nb_update++;

            checkNewExplanation(task->getName(), &msg_finished_task_);

            callback_output_(msg_finished_task_);

            if (nb_update != 0)
                for (auto& task_: tasks_)
                {
                    task_->clean();
                }

        } while (nb_update != 0);
    }
    finished_task_.clear();

}
void TaskRecognition::updateActionMethod(MessageType type, procedural::ActionMethod* action_method)
{
    if (type == MessageType::Finished or type == MessageType::Complete)
        finished_actions_method_.push_back(action_method);
    if (type == MessageType::Update)
        updated_actions_method_.push_back(action_method);
}
void TaskRecognition::updateAction(MessageType type, Action* action)
{
    if (type == MessageType::Finished or type == MessageType::Complete)
        finished_actions_.push_back(action);
    if (type == MessageType::Update)
        updated_actions_.push_back(action);
}
void TaskRecognition::updateTask(MessageType type, Task* task)
{
    if (type == MessageType::Finished or type == MessageType::Complete)
        finished_task_.push_back(task);
    if (type == MessageType::Update)
        updated_task_.push_back(task);
}
void TaskRecognition::defaultCallback(const std::vector<TaskRecognizedMSG_t>& outputs)
{
    for (const auto& output: outputs)
        std::cout << ">> " << output << std::endl;

}
void TaskRecognition::checkNewExplanation(const std::string& task_name,std::vector<TaskRecognizedMSG_t>* msgs)
{
    for (auto& updated_task: updated_task_)
    {
        auto nets = updated_task->getNewExplanation();
        for (auto& net: nets)
            msgs->emplace_back(updated_task->getName(), net);
    }
    for (auto& updated_action_method: updated_actions_method_)
    {
        auto nets = updated_action_method->getNewExplanation();
        for (auto& net: nets)
            msgs->emplace_back(task_name,net);
    }
    for (auto& updated_actions: updated_actions_)
    {
        auto nets = updated_actions->getNewExplanation();
        for (auto& net: nets)
            msgs->emplace_back(task_name,net);
    }

    updated_task_.clear();
    updated_actions_method_.clear();
    updated_actions_.clear();

}
} // procedural