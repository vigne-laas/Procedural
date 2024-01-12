#include <vector>
#include "procedural/core/TaskRecognition.h"
#include "procedural/core/Types/Task.h"
#include "procedural/core/Types/ActionMethod.h"
#include "procedural/core/Types/Action.h"
namespace procedural {

void TaskRecognition::init(const std::vector<Task*>& tasks, double ttl)
{

    tasks_ = tasks;
    for (auto task: tasks_)
        std::cout << task->getName() << std::endl;
    callback_output_ = TaskRecognition::defaultCallback;
}
void TaskRecognition::process(TimeStamp_t current_time)
{
    std::vector<TaskRecognizedMSG_t> msg_finished_task_;
    for (auto task: tasks_)
    {
        std::cout << "current task process " << task->getName() << std::endl;
        for (const auto& f_action: finished_actions_)
        {
//            std::cout << "\t\t action : " << f_action->getName() << std::endl;
            auto feed_result = task->feed(f_action);
            for (auto finish_method: feed_result.finished_)
            {
                finished_task_.push_back(task);
                msg_finished_task_.emplace_back(task->getName(), finish_method);

            }

        }

        int nb_update;
        do
        {
            nb_update = 0;
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

//void TaskRecognition::updateAction(MessageType type, Action* action)
//{
//    if (type == MessageType::Finished or type == MessageType::Complete)
//        finished_actions_.push_back(action);
//    if (type == MessageType::Update)
//        updated_actions_.push_back(action);
//}
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
void TaskRecognition::checkNewExplanation(const std::string& task_name, std::vector<TaskRecognizedMSG_t>* msgs)
{
    for (auto& updated_task: updated_task_)
    {
        auto nets = updated_task->getNewExplanation();
        for (auto& net: nets)
            msgs->emplace_back(updated_task->getName(), net);
    }
//    for (auto& updated_actions: updated_actions_)
//    {
//        auto nets = updated_actions->getNewExplanation();
//        for (auto& net: nets)
//            msgs->emplace_back(task_name, net);
//    }

    updated_task_.clear();
//    updated_actions_method_.clear();
    updated_actions_.clear();

}
void TaskRecognition::actionEvent(ActionEvent_t event)
{
    std::cout << "event receive " << event.sm_finish->getName() << std::endl;
    if (event.state_ == FeedResult::FINISH)
        finished_actions_.push_back(event.sm_finish);
    if (event.state_ == FeedResult::NEW_EXPLANATION)
        updated_actions_.push_back(event.sm_finish);
}
} // procedural