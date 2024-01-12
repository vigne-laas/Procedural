#ifndef PROCEDURAL_TASKRECOGNIZEDMSG_H
#define PROCEDURAL_TASKRECOGNIZEDMSG_H
#include <cstdio>
#include "procedural/utils/TimeStamp.h"
#include "procedural/core/Types/Method.h"
namespace procedural {
class Method;

struct TaskRecognizedMSG_t
{
    TaskRecognizedMSG_t(const std::string& name, Method* method) : name_(method->getName()), updated(false)
    {
        //TODO complete to extract finish method and set all variables need in the message
    }

    friend std::ostream& operator<<(std::ostream& os, const TaskRecognizedMSG_t& val)
    {
        if (val.updated)
            os << "new explanation for : ";
        else
            os << "finished task : ";
        os << val.name_;//<< " [ " << val.start_time << " ; " << val.stop_time << " ] ";
//        if (val.descriptions.empty() == false)
//        {
//            os << " descriptions : ";
//            for (auto& description: val.descriptions)
//                os << description << " | ";
//        }

        return os;
    }

    std::string name_;
    bool updated;
    TimeStamp_t start_time;
    TimeStamp_t stop_time;


};
}

#endif //PROCEDURAL_TASKRECOGNIZEDMSG_H
