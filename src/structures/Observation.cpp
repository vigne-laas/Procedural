#include "procedural/structures/Observation.h"
#include "procedural/structures/ObservationFact.h"
#include "procedural/action_recognition/core/internal_structures/Action.h"

namespace procedural {

void Observation::linkVariables(std::map<std::string, std::shared_ptr<Variable_t>>& variables)
{
//    LOG_INFO << "Observation::linkVariables";
    for (auto& var: variables) {
        table_variables_.variables[var.first] = var.second;
    }
}

bool Observation::operator==(const Observation& other)
{
    LOG_INFO << "Observation::operator==";
    if (id_ == other.id_) {
        bool match = true;
        std::map<std::string, std::shared_ptr<Variable_t>> to_add_variables;
        if (other.table_variables_.variables.empty())
            return false;
        for (auto& var: other.table_variables_.variables) {
            LOG_DEBUG << "Checking variable " << var.first << " in observation " << id_ << " with value "
                      << var.second->value_;
            if (match) {
                if (table_variables_.variables.find(var.first) == table_variables_.variables.end())
                    match = false;
                else {
                    if (!table_variables_.variables[var.first]->isSet()) {
                        match = true;
                        LOG_DEBUG << "Unset variable " << var.first << " in observation " << id_ << " with value "
                                  << var.second->value_;
                        to_add_variables.insert(std::make_pair(var.first, var.second));
                    } else {
                        if (table_variables_.variables[var.first] != var.second)
                            match = false;
                    }
                }
            }

        }
        if (match) {
            for (auto& var: to_add_variables) {
                LOG_INFO << "Adding variable " << var.first << " to observation " << id_ << " with value "
                         << var.second->value_;
                table_variables_.variables[var.first] = var.second;
            }
        }
        return match;
    }
    return false;
}

Observation::Observation(const Observation& other)
{
    this->id_ = other.id_;
    for (auto& var: other.table_variables_.variables) {
        auto new_var = std::make_shared<Variable_t>(*var.second);
        new_var->value_ = var.second->value_;
        table_variables_.variables[var.first] = new_var;
    }
    for (auto& agent: other.table_variables_.agents) {
        table_variables_.agents.insert(agent);
    }


}

std::ostream& operator<<(std::ostream& os, const Observation& obs)
{
    os << obs.toString();
    return os;
}

std::string Observation::toString() const
{
    std::string result = "Observation :" + std::to_string(id_) + "\n";
    result += Action::table_actions_[id_];
    if (!table_variables_.variables.empty())
        result += "\n"+table_variables_.toString();
    if(!remap_.empty())
    {
        result += "\t Remap:\n";
        for(auto& remap : remap_)
        {
            result += remap.first + " -> " + remap.second + "\n";
        }
    }

    return result;
}

void Observation::setRemap(std::map<std::string, std::string> remap_param)
{
    remap_ = remap_param;
}

void Observation::completeVar(const std::vector<std::shared_ptr<Action>>& actions)
{
    auto res = std::find_if(actions.begin(), actions.end(), [this](const std::shared_ptr<Action>& action) {
        return action->getId() == id_;
    });
    if (res != actions.end()) {
        for(const auto& var : (*res)->getFactory()->getTableVariables().variables) {
            if (table_variables_.variables.find(var.first) == table_variables_.variables.end()) {
                if(var.first != "self")
                {
                    table_variables_.variables[var.first] = std::make_shared<Variable_t>(var.second->literal_);
                    LOG_DEBUG << "Adding variable " << var.first << " to observation " << id_ << " with value "
                             << var.second->toString();
                }
            }
        }
    }
    else {
        LOG_ERROR << "No action found for observation " << id_ << "\n";
        throw std::runtime_error("No action found for observation " + std::to_string(id_));
    }


}


} // procedural