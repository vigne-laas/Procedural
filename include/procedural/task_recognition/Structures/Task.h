#ifndef PROCEDURAL_TASK_H
#define PROCEDURAL_TASK_H

#include <string>
#include "procedural/task_recognition/Reader/domainTypes/ParsedHTN.h"
#include "procedural/task_recognition/Structures/Method.h"

namespace procedural {

class Task {
public :
    Task(const std::string& name) : name_(name){};

    bool build(const Abstract_task_t& task);
    bool evolve(Observation* obs);

    std::vector<Method*> getActiveMethods() const { return active_methods_; };
    std::vector<Method*> getFactoryMethods() const { return factory_methods_; };
    std::vector<Method*> getFinishedMethods() const { return finished_methods_; };

    std::string getName() const { return name_; };

private:
    std::string name_;
    int method_id_;
    std::vector<Method*> factory_methods_;
    std::vector<Method*> active_methods_;
    std::vector<Method*> finished_methods_;


};

} // procedural

#endif //PROCEDURAL_TASK_H
