#ifndef PROCEDURAL_VARIABLESTABLE_H
#define PROCEDURAL_VARIABLESTABLE_H

#include <map>
#include <string>
#include <unordered_set>
#include <memory>
#include "procedural/structures/Variable.h"
#include "procedural/structures/Fact.h"
#include "procedural/utils/WordTable.h"
//#include "procedural/structures/Observation.h"

namespace procedural {
class Observation;

class ObservationFact;

struct VariableTable_t {
    VariableTable_t() = default;

    VariableTable_t(const VariableTable_t& other)
    {
        for (const auto& variable: other.variables)
            variables[variable.first] = std::make_shared<Variable_t>(*variable.second);
        agents = other.agents;
        remap_ = other.remap_;
    }

    std::map<std::string, std::shared_ptr<Variable_t>> variables;
    std::unordered_set<std::string> agents;
    std::map<std::string, std::string> remap_;

    void set(VariableTable_t other)
    {
//        LOG_DEBUG << "------------- Set VariableTable ------------------";
//        LOG_DEBUG << "Origin Graph: " << toString();
//        LOG_DEBUG << "Other nouvelle transition: " << other.toString();

        for (const auto& variable: other.variables) {
            if (other.remap_.find(variable.first) != other.remap_.end()) {
                if (variables.find(other.remap_.at(variable.first)) == variables.end()) {
//                    LOG_DEBUG << "Set variable " << other.remap_.at(variable.first) << "with remap to "
//                              << variable.second->toString();
//                    LOG_DEBUG << "remap from " << variable.first << " to " << other.remap_.at(variable.first);
                    variables[other.remap_.at(variable.first)] = variable.second;
                }
            } else {
                if (variables.find(variable.first) == variables.end()) {
//                    LOG_DEBUG << "Set variable " << variable.first << " to " << variable.second->toString();
                    variables[variable.first] = variable.second;
                }

            }
        }
        remap_.insert(other.remap_.begin(), other.remap_.end());
//        LOG_DEBUG << "After set";
//        LOG_DEBUG << "Graph : " << toString();
//        LOG_DEBUG << "transition" << other.toString();
//        LOG_DEBUG << "---------------- END Set VariableTable ------------------";

//        for (const auto& agent: other.agents) {
//            agents.insert(agent);
//        }
    }

    void setRemap(std::map<std::string, std::string> remap_param)
    {
        remap_ = remap_param;
    }

    bool operator==(const VariableTable_t& other) const
    {
//        LOG_DEBUG << "Compare VariableTable";
        if (variables.size() != other.variables.size()) {
            LOG_DEBUG << "Variables size are different";
            return false;
        }
        for (const auto& variable: variables) {
//            LOG_DEBUG << "Variable : " << variable.first;
            if (other.variables.find(variable.first) == other.variables.end()) {
//                LOG_DEBUG << "Variable " << variable.first << " not found in other table";
                if (remap_.find(variable.first) != remap_.end()) {
//                    LOG_DEBUG << "Variable " << variable.first << " is remapped to " << remap_.at(variable.first);
                    if (other.variables.find(remap_.at(variable.first)) == other.variables.end()) {
                        LOG_ERROR << "Variable " << variable.first << "remap as :" << remap_.at(variable.first)
                                  << " not found in other table";
                        return false;
                    } else {
                        if (!variable.second->match(*other.variables.at(remap_.at(variable.first)))) {
//                            LOG_DEBUG << "Variable " << variable.first << "remap as :" << remap_.at(variable.first)
//                                      << " not equal in other table : "
//                                      << other.variables.at(remap_.at(variable.first))->toString() << " != "
//                                      << variable.second->toString();
                            return false;
                        }
//                        else {
//                            LOG_DEBUG << "Variable " << variable.first << "remap as :" << remap_.at(variable.first)
//                                      << " equal in other table : "
//                                      << other.variables.at(remap_.at(variable.first))->toString() << " == "
//                                      << variable.second->toString();
//                        }
                    }
                } else {
                    LOG_ERROR << "Variable " << variable.first << " not found in other table even with remap";
                    return false;
                }
            }
            if (!variable.second->match(*other.variables.at((variable.first)))) {
                LOG_DEBUG << "Variable " << variable.first << " not equal in other table : "
                          << other.variables.at(variable.first)->toString() << " != " << variable.second->toString();
                return false;
            }
//            else {
//                LOG_DEBUG << "Variable " << variable.first << " equal in other table : "
//                          << other.variables.at(variable.first)->toString() << " == " << variable.second->toString();
//            }

        }
        return true;
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
                str += variable.second->toString(variable.first);
                str += "[" + std::to_string(reinterpret_cast<uintptr_t>(variable.second.get())) + "]";
                str += "\n";
            }
        }
        if (!agents.empty()) {
            str += "agents : \n";
            for (const auto& agent: agents) {
                str += agent + "\n";
            }
        }
        if (!remap_.empty()) {
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
