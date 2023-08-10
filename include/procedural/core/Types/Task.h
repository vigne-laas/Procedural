#ifndef PROCEDURAL_TASK_H
#define PROCEDURAL_TASK_H

#include <string>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

#include "procedural/core/Types/Action.h"
#include "procedural/core/Types/Method.h"


namespace procedural {

class Task
{
public:
    explicit Task(std::string name) : name_(name), id_(task_types.get(name)){};

    void addMethods(const Method& method);
    void addArguments(const std::map<std::string, std::string>& arguments);

    bool feed(Action* action);
    bool feed(Task* task);

    std::string getName() { return name_; }
    std::vector<Method> getMethods() { return methods_; };
    uint32_t getId() { return id_; };

    static WordTable task_types;
private:
    uint32_t id_;
    std::map<std::string, std::string> arguments_;
    std::string name_;
    std::vector<Method> methods_;
//    std::vector<>

    std::unordered_set<Task*> complete_task_;
};

} // procedural

#endif //PROCEDURAL_TASK_H
