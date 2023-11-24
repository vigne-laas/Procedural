#ifndef PROCEDURAL_TASKRECOGNIZEDMSG_H
#define PROCEDURAL_TASKRECOGNIZEDMSG_H
#include <cstdio>

namespace procedural {
struct TaskRecognizedMSG_t
{
    TaskRecognizedMSG_t(const std::string& task_name, Method* method) : name_(task_name)
    {

    }
    TaskRecognizedMSG_t(const std::string& task_name, ActionMethod* action_method) : name_(task_name)
    {

    }
    TaskRecognizedMSG_t(const std::string& task_name, Action* action) : name_(task_name)
    {

    }
    TaskRecognizedMSG_t(const std::string& task_name, StateMachine* sm) : name_(task_name)
    {

    }


    friend std::ostream& operator<<(std::ostream& os, const TaskRecognizedMSG_t& val)
    {
//        if (val.updated)
//            os << "new explanation for : ";
//        else
//            os << "finished : ";
//        os << val.name << " [ " << val.start_time << " ; " << val.stop_time << " ] ";
//        if (val.descriptions.empty() == false)
//        {
//            os << " descriptions : ";
//            for (auto& description: val.descriptions)
//                os << description << " | ";
//        }

        return os;
    }

    std::string name_;
};
}

#endif //PROCEDURAL_TASKRECOGNIZEDMSG_H
