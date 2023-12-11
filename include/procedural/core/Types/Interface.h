#ifndef PROCEDURAL_INTERFACE_H
#define PROCEDURAL_INTERFACE_H
#include <string>
namespace procedural {

class Action;
class Method;
class Task;

enum class MessageType
{
    None = 0,
    Update = 1,
    Finished = 2,
    Complete = 3
};


class IObserver
{
public:
    virtual ~IObserver() = default;
    virtual void updateAction(MessageType type, Action* action) {};
    virtual void updateMethod(MessageType type, Method* method) {};
    virtual void updateTask(MessageType type, Task* task) {};
};

class ISubject
{
public:
    virtual ~ISubject() = default;
    virtual void attach(IObserver* observer) = 0;
    virtual void detach(IObserver* observer) = 0;
    virtual void notify(MessageType type) = 0;
};
}

#endif //PROCEDURAL_INTERFACE_H
