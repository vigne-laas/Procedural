#ifndef PROCEDURAL_METHOD_H
#define PROCEDURAL_METHOD_H

#include "procedural/core/Graph/StateMachine.h"
#include "procedural/core/Types/HTNTransition.h"
#include "procedural/core/Types/Action.h"
namespace procedural {

class Method
{
public:
    Method(const std::string& name, int id);
    void addTransition(const HTNTransition_t& transition);
    void close();
    std::string getStrStructure();
    EvolveResult_t feed(StateMachine* sm);
    EvolveResult_t feed(Task* task);

    int getNextSMId() { return compt_SM_++; }
    std::string getName() { return name_; }

    void saveDot(int i,const std::string& suffix = "",bool partial= false);
private:

    std::string name_;
    int id_;
    int compt_SM_;
    StateMachine* factory_machine_;

    std::unordered_set<StateMachine*> current_state_machines_;
    std::unordered_set<StateMachine*> finished_state_machines_;
    std::vector<StateMachine*> updated_states_machines;
    


};

} // procedural

#endif //PROCEDURAL_METHOD_H
