#ifndef PROCEDURAL_METHOD_H
#define PROCEDURAL_METHOD_H

#include "procedural/core/Graph/StateMachine.h"
#include "procedural/core/Types/HTNTransition.h"
namespace procedural {

class Method : IObserver, ISubject
{
public:
    Method(const std::string& name, int id);
    void addTransition(const HTNTransition_t& transition);
    void close();
    std::string getStrStructure();
    bool feed(Action* action);
    bool feed(Task* task);
    bool feed(ActionMethod* action_method);
    int getNextSMId() { return compt_SM_++; }


    void attach(IObserver* observer) override
    {
        list_observer_.push_back(observer);
    }
    void detach(IObserver* observer) override
    {
        list_observer_.remove(observer);
    }
    void updateStateMachine(MessageType type, StateMachine* machine);


    void notify(MessageType type) override;

private:

    std::string name_;
    int id_;
    int compt_SM_;
    StateMachine* factory_machine_;

    std::unordered_set<StateMachine*> current_state_machines_;
    std::unordered_set<StateMachine*> finished_state_machines_;
    std::vector<StateMachine*> updated_states_machines;

    MessageType notify_type_;
    std::list<IObserver*> list_observer_;


};

} // procedural

#endif //PROCEDURAL_METHOD_H
