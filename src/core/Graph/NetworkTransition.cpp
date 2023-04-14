#include "procedural/core/Graph/NetworkTransition.h"
#include "procedural/core/Graph/Network.h"

namespace procedural {

WordTable NetworkTransition::sub_network_table;

NetworkTransition::NetworkTransition(uint32_t type, const std::map<std::string, std::string>& remap_var) : type_(type),
                                                                                                           remap_var_(
                                                                                                                   remap_var)
{
    for (const auto& pair : remap_var_)
        variables_.insert(std::make_pair(pair.second,
                                         nullptr)); // faire lien avec le constructeur de transition fact pour pouvoir lier les variables dans le resseaux.
}

std::string NetworkTransition::toString() const
{
    std::string res = "Network Transition type : " + std::to_string(type_) + "(" + sub_network_table.get(type_) + ")\n";
    for (auto& pair : remap_var_)
        res += "\t" + pair.first + " ==> " + pair.second + "\n";
    res += "Variables : \n";
    for (auto& var : variables_)
        res += "\t" + var.first + " : " + var.second->toString() + "\n";
    return res;
}

bool NetworkTransition::match(Network* net)
{
    auto type = sub_network_table.getConst(net->getType());
    if (type == type_)
    {
        for (const auto& pair : remap_var_)
        {
            uint32_t local_val = variables_.at(pair.second)->getValue();
            if(local_val == 0)
                variables_.at(pair.second)->value = net->getVar(pair.first).getValue();
            else if (net->getVar(pair.first).getValue() != local_val)
                return false;
        }
        return true;
    }
    else
        return false;
}

void NetworkTransition::linkVariables(std::map<std::string, Variable_t>& variables)
{
    for (auto& pair: variables_)
        pair.second = &(variables.at(pair.first));
}


} // procedural