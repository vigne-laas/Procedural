#ifndef PROCEDURAL_METHOD_H
#define PROCEDURAL_METHOD_H

#include "procedural/core/Graph/StateMachine.h"
#include "procedural/core/Types/HTNTransition.h"
namespace procedural {

class Method
{
public:
    Method(const std::string& name, int id);
    void addTransition(const HTNTransition_t& transition);
    void close();
    std::string getStrStructure();
    bool feed(Action* action);
    bool feed(Task* task);
    bool feed(ActionMethod* action_method);
    int getNextSMId() {return compt_SM_++;}

private:

    std::string name_;
    int id_;
    int compt_SM_;
    StateMachine* factory_machine_;
    std::unordered_set<StateMachine*> current_state_machines_;


};

} // procedural

#endif //PROCEDURAL_METHOD_H
