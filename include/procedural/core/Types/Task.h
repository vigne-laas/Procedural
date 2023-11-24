#ifndef PROCEDURAL_TASK_H
#define PROCEDURAL_TASK_H

#include <string>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

#include "procedural/core/Types/Action.h"
#include "procedural/core/Types/Method.h"


namespace procedural {

class Task : public ISubject, IObserver
{
public:
    explicit Task(std::string name) : name_(name), id_(task_types.get(name)) {};

    void addMethods(const Method& method);
    void addArguments(const std::map<std::string, std::string>& arguments);

    bool feed(Action* action);
    bool feed(Task* task);
    bool feed(ActionMethod* action_method);

    std::string getName() { return name_; }
    std::vector<Method> getMethods() { return methods_; };
    uint32_t getId() { return id_; };

    static WordTable task_types;
    std::unordered_set<Method*> getFinishedMethods() { return finished_methods_; }
    void clean();

    void updateMethod(MessageType type, Method* method) override;

    void attach(IObserver* observer) override
    {
        list_observer_.push_back(observer);
    }
    void detach(IObserver* observer) override
    {
        list_observer_.remove(observer);
    }

    void notify(MessageType type) override;
    std::unordered_set<Method*> getNewExplanation() { return updated_methods_;}
private:
    uint32_t id_;
    std::map<std::string, std::string> arguments_;
    std::string name_;
    std::vector<Method> methods_;
//    std::vector<>

    std::unordered_set<Method*> finished_methods_;
    std::unordered_set<Method*> updated_methods_;

    std::map<std::string, Variable_t> getVars();

    std::list<IObserver*> list_observer_;
    MessageType notify_type_;
};

} // procedural

#endif //PROCEDURAL_TASK_H
