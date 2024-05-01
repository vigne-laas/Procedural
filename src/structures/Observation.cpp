#include "procedural/structures/Observation.h"
#include "procedural/structures/ObservationFact.h"
#include "procedural/action_recognition/core/internal_structures/Action.h"

namespace procedural {

void Observation::linkVariables(std::map<std::string, std::shared_ptr<Variable_t>>& variables)
{
//    LOG_INFO << "Observation::linkVariables in " << id_;
//    LOG_INFO << "LOCAL VARIABLES";
//    LOG_INFO << table_variables_.toString();

    for (auto& var: table_variables_.variables) {
        if (var.first != "self") {
            if (table_variables_.remap_.find(var.first) != table_variables_.remap_.end()) {
//                LOG_DEBUG << "Remap : " << var.first << " -> " << remap_[var.first];
//                LOG_DEBUG << "General Var :" << var.first << " -> " << *variables[remap_[var.first]];
                var.second = variables[table_variables_.remap_[var.first]];
            } else {
//                LOG_DEBUG << "General Var :" << var.first << " -> " << *variables[var.first];
                var.second = variables[var.first];
            }
        }
    }

}

bool Observation::operator==(const Observation& other)
{
//    LOG_INFO << "Observation::operator==";
//    LOG_DEBUG << "left observation : " << toString();
//    LOG_DEBUG << "right observation : " << other.toString();
    if (id_ == other.id_) {
        bool match = true;
        std::map<std::string, std::shared_ptr<Variable_t>> to_add_variables;
        if (table_variables_ == other.table_variables_) {
//            LOG_INFO << "Variables are equals";
            return true;
        } else {
//            LOG_INFO << "Variables are not equals";
            return false;
        }
//        if (match) {
//            for (auto& var: to_add_variables) {
//                LOG_INFO << "Adding variable " << var.first << " to observation " << id_ << " with value "
//                         << var.second->value_;
//                table_variables_.variables[var.first] = var.second;
//            }
//        }
//        return match;
    }
    return false;
}

Observation::Observation(const Observation& other)
{
    this->id_ = other.id_;
//    LOG_DEBUG << "Copy constructor Observation " << id_;
//    LOG_DEBUG << "other table_variables_ : " << other.table_variables_.toString();
    for (auto& var: other.table_variables_.variables) {
        auto new_var = std::make_shared<Variable_t>(*var.second);
        new_var->value_ = var.second->value_;
        table_variables_.variables[var.first] = new_var;
    }
    table_variables_.remap_ = other.table_variables_.remap_;
//    remap_ = other.remap_;
    for (auto& agent: other.table_variables_.agents) {
        table_variables_.agents.insert(agent);
    }
//    LOG_DEBUG << "new table_variables_ after copy: " << table_variables_.toString();


}

std::ostream& operator<<(std::ostream& os, const Observation& obs)
{
    os << obs.toString();
    return os;
}

std::string Observation::toString() const
{
    std::string result = "Observation :" + std::to_string(id_) + "\n";
    result += WordTable::actions_table[id_];
    if (!table_variables_.variables.empty())
        result += "\n" + table_variables_.toString();
//    if (!remap_.empty()) {
//        result += "\t Remap:\n";
//        for (auto& remap: remap_) {
//            result += remap.first + " -> " + remap.second + "\n";
//        }
//    }

    return result;
}

void Observation::setRemap(std::map<std::string, std::string> remap_param)
{
//    remap_ = remap_param;
    table_variables_.setRemap(remap_param);
}

void Observation::completeVar(const std::vector<std::shared_ptr<Action>>& actions)
{
    LOG_INFO << "Observation::completeVar";
    auto res = std::find_if(actions.begin(), actions.end(), [this](const std::shared_ptr<Action>& action) {
        return action->getId() == id_;
    });
    if (res != actions.end()) {
        for (const auto& var: (*res)->getFactory()->getTableVariables().variables) {
            if (table_variables_.variables.find(var.first) == table_variables_.variables.end()) {
                if (var.first != "self") {
                    table_variables_.variables[var.first] = std::make_shared<Variable_t>(*var.second);
                    LOG_DEBUG << "Adding variable " << var.first << " to observation " << id_ << " with value "
                              << var.second->toString();
                }
            }
        }
    } else {
        LOG_ERROR << "No action found for observation " << id_ << "\n";
        throw std::runtime_error("No action found for observation " + std::to_string(id_));
    }


}


} // procedural