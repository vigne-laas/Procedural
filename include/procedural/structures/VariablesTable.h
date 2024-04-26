#ifndef PROCEDURAL_VARIABLESTABLE_H
#define PROCEDURAL_VARIABLESTABLE_H

#include <map>
#include <string>
#include <unordered_set>
#include <memory>
#include "procedural/structures/Variable.h"
#include "procedural/structures/Fact.h"
//#include "procedural/structures/Observation.h"

namespace procedural {
class Observation;
class ObservationFact;
struct VariableTable_t {
    VariableTable_t() = default;

    std::map<std::string, std::shared_ptr<Variable_t>> variables;
    std::unordered_set<std::string> agents;
    std::map<std::string,std::string> remap_;

    void set(VariableTable_t other)
    {
        for (const auto& variable: other.variables) {
            if (variables.find(variable.first) == variables.end()) {
                variables[variable.first] = variable.second;
            }
        }
        for (const auto& agent: other.agents) {
            agents.insert(agent);
        }
    }

//    void update(Observation* observation)
//    {
//        if (observation->getId() < 0) {
//            const auto* observation_fact = dynamic_cast<ObservationFact*>(observation);
//            if (observation_fact) {
//
//            }
//
//            return;
//        }
//    }



    std::string toString() const
    {
        std::string str;
        if (!variables.empty()) {
            str += "variables : \n";
            for (const auto& variable: variables) {
                str += variable.first + " : " + variable.second->toString() +
                       (variable.second->isSet() ? " : " + Fact::individuals_table[variable.second->value_] : "") +
                       "\n";
            }
        }
        if (!agents.empty()) {
            str += "agents : \n";
            for (const auto& agent: agents) {
                str += agent + "\n";
            }
        }
        if(!remap_.empty())
        {
            str += "remap : \n";
            for (const auto& remap: remap_) {
                str += remap.first + " : " + remap.second + "\n";
            }
        }
        return str;
    }
};
} // namespace procedural
#endif //PROCEDURAL_VARIABLESTABLE_H
