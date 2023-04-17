#ifndef PROCEDURAL_NETWORKTRANSITION_H
#define PROCEDURAL_NETWORKTRANSITION_H

#include <string>
#include <map>

#include "procedural/core/Types/WordTable.h"
#include "procedural/core/Types/Variable.h"

namespace procedural {

class Network;

class TransitionNetwork
{
public:
    TransitionNetwork(uint32_t type, const std::map<std::string,std::string>& remap_var);

    std::string toString() const ;
    bool match(Network * network) ;
    void linkVariables(std::map<std::string, Variable_t>& variables);

private:
    uint32_t type_;
    std::map<std::string,std::string> remap_var_;
    std::map<std::string,Variable_t*> variables_;
};

} // procedural

#endif //PROCEDURAL_NETWORKTRANSITION_H
