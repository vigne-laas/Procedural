#ifndef PROCEDURAL_TASKRECOGNITION_H
#define PROCEDURAL_TASKRECOGNITION_H

#include "procedural/core/Types/Interface.h"
#include "procedural/core/Types/TaskRecognizedMSG.h"
#include "procedural/utils/TimeStamp.h"
#include <functional>
namespace procedural {

class Action;
class ActionMethod;
class Task;

class TaskRecognition : public IObserver
{
public:
    TaskRecognition() = default;
    void init(const std::vector<Task*>& tasks, double ttl = 25);

    void process(TimeStamp_t current_time);

    void setCallback(const std::function<void(
            const std::vector<TaskRecognizedMSG_t>&)>& callback) { callback_output_ = callback; }

//    void updateAction(MessageType type, Action* machine) override;
    void updateTask(MessageType type, Task* task) override;
    void actionEvent(ActionEvent_t event) override;


private:
    static void defaultCallback(const std::vector<TaskRecognizedMSG_t>& outputs);
    std::vector<Task*> tasks_;

    std::function<void(const std::vector<TaskRecognizedMSG_t>&)> callback_output_;
    std::vector<StateMachine*> finished_actions_;
    std::vector<StateMachine*> updated_actions_;
    std::vector<Task*> finished_task_;
    std::vector<Task*> updated_task_;


    void checkNewExplanation(const std::string& task_name,std::vector<TaskRecognizedMSG_t>* msgs);
};

} // procedural

#endif //PROCEDURAL_TASKRECOGNITION_H
