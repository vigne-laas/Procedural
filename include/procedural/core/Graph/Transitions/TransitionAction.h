#ifndef PROCEDURAL_TRANSITIONSTATEMACHINE_H
#define PROCEDURAL_TRANSITIONSTATEMACHINE_H

#include <string>
#include <map>

#include "procedural/core/Types/Variable.h"

namespace procedural {

class StateMachine;

class TransitionAction
{
public:
    TransitionAction(uint32_t type, const std::map<std::string,std::string>& remap_var);

    void linkVariables(std::map<std::string, Variable_t>& variables);

    bool match(StateMachine * stateMachine) ;

    bool checkMatch(){return flag_;}
    std::map<std::string,std::string> getRemap(){return remap_var_;}
    
    std::string toString() const ;
    
private:
    uint32_t type_;
    bool flag_;
    std::map<std::string,std::string> remap_var_;
    std::map<std::string,Variable_t*> variables_;
};

} // namespace procedural

#endif // PROCEDURAL_TRANSITIONSTATEMACHINE_H
