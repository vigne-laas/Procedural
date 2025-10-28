#ifndef PROCEDURAL_PARSEDHTN_H
#define PROCEDURAL_PARSEDHTN_H

#include <vector>
#include <string>
#include <iostream>
#include <set>
#include <map>
#include "procedural_interfaces/Task.h"
#include "procedural_interfaces/Method.h"
#include "procedural_interfaces/TaskArgument.h"
#include "procedural_interfaces/TaskPrecondition.h"
#include "procedural_interfaces/TaskEffect.h"

namespace procedural {
struct Expression_t {
    Expression_t() = default;

    Expression_t(const std::string& subject, const std::string& property, const std::string& object) : subject(subject),
                                                                                                       property(
                                                                                                               property),
                                                                                                       object(object) {};
    std::string subject;
    std::string property;
    std::string object;
    bool add = true;

    friend std::ostream& operator<<(std::ostream& os, const Expression_t& lhs)
    {
        os << lhs.subject << " " << lhs.property << " " << lhs.object;
        return os;
    }
};


struct Arguments_t {
    Arguments_t() = default;
    Arguments_t(const std::string& type, const std::string& varname) : type(type), varname(varname), name(varname) {};
    std::string type;
    std::string varname;
    std::string name;

    friend std::ostream& operator<<(std::ostream& os, const Arguments_t& lhs)
    {
        os << lhs.type << " " << lhs.varname;
        return os;
    }

};

struct Preconditions_t {

    friend std::ostream& operator<<(std::ostream& os, const Preconditions_t& lhs)
    {
        return os;
    }
};

struct Selection_t {
    Selection_t(const std::string& attribut, const std::string& type, const Expression_t& expression) : attribut(
            attribut), type(type), expression(expression) {};
    std::string attribut;
    std::string type;
    Expression_t expression;

    friend std::ostream& operator<<(std::ostream& os, const Selection_t& lhs)
    {
        os << lhs.attribut << " : " << lhs.type << " = " << lhs.expression;
        return os;
    }

};

struct Ordered_Action_t {

    Ordered_Action_t() = default;

    int id = 0;
    std::string name = " ";
    std::vector<std::string> arguments = {};
    std::set<int> after_id = {};
//    std::vector<int> link_to;

    friend std::ostream& operator<<(std::ostream& os, const Ordered_Action_t& lhs)
    {
        os << lhs.id << " " << lhs.name;
        os << "(";
        for (auto arg: lhs.arguments)
            os << arg << ",";
        os << ")";
        os << " after ";
        for (auto id_: lhs.after_id)
            os << id_ << ",";
        return os;
    }

};

struct Subtask_t {
    std::vector<Selection_t> selections;
//    std::vector<Ordered_Action_t> actions_;
    std::map<int, Ordered_Action_t> map_actions;

    friend std::ostream& operator<<(std::ostream& os, const Subtask_t& lhs)
    {
        os << "subtask : \n";
        for (const auto& select: lhs.selections)
            os << "\t\t" << select << "\n";
        for (const auto& action: lhs.map_actions)
            os << "\t" << action.first << " : " << action.second << "\n";
        return os;
    }

    void linkActions()
    {
        for (auto& action: map_actions) {
            for (auto& id: action.second.after_id) {
                action.second.after_id.insert(map_actions[id].after_id.begin(), map_actions[id].after_id.end());
            }
        }
    }
};

struct Method_t {
    std::string name;
    std::vector<Expression_t> preconditions;
    Subtask_t subtask;

    friend std::ostream& operator<<(std::ostream& os, const Method_t& lhs)
    {
        os << "Method: " << lhs.name << "\n";
        os << "Decomposition  : \n";
        for (const auto& precondition: lhs.preconditions)
            os << "\t\t" << precondition << "\n";
        os << "\t\t" << lhs.subtask;
        return os;
    }
};

struct effects_t {
    std::vector<std::string> other_effects;
    std::vector<Expression_t> simple_effects;

    friend std::ostream& operator<<(std::ostream& os, const effects_t& lhs)
    {
        os << ((lhs.other_effects.empty()) ? "" : "complexe effects : ");
        for (const auto& effect: lhs.other_effects)
            os << "\t -" << effect << "\n";
        os << ((lhs.simple_effects.empty()) ? "" : "simple effects : ");
        for (const auto& effect: lhs.simple_effects)
            os << "\t -" << effect << "\n";
        return os;
    }
};

struct Abstract_task_t {
    std::string name;
    std::vector<Expression_t> goals;
    std::vector<Arguments_t> arguments;
    std::vector<Method_t> methods_;
    effects_t effects;

    friend std::ostream& operator<<(std::ostream& os, const Abstract_task_t& lhs)
    {
        os << "Method : " << lhs.name << "\n";

        os << "\t Goal : \n";
        for (const auto& goal: lhs.goals)
            os << "\t\t" << goal << "\n";
        for (const auto& arg: lhs.arguments)
            os << "\t" << arg << "\n";
        for (const auto& decomposition: lhs.methods_)
            os << "\t" << decomposition << "\n";
        os << "\t" << lhs.effects;
        return os;
    }

    procedural_interfaces::Task toRosMsg() const
    {
        procedural_interfaces::Task msg;
        msg.task_name = name;

        // Convert arguments
        for (const auto& arg : arguments)
        {
            procedural_interfaces::TaskArgument task_arg;
            task_arg.name = arg.name;
            task_arg.type = arg.type;
            task_arg.value = "";  // No value in parsed structure
            msg.arguments.push_back(task_arg);
        }

        // Convert methods - full conversion with decomposition
        for (const auto& method : methods_)
        {
            procedural_interfaces::Method method_msg;
            method_msg.method_name = method.name.empty() ?
                ("method_" + std::to_string(&method - &methods_[0])) :
                method.name;

            // Convert preconditions
            for (const auto& precond : method.preconditions)
            {
                procedural_interfaces::TaskPrecondition precond_msg;
                precond_msg.subject = precond.subject;
                precond_msg.predicate = precond.property;
                precond_msg.object = precond.object;
                precond_msg.is_negative = !precond.add;
                method_msg.preconditions.push_back(precond_msg);
            }

            // Convert subtasks to decomposition array
            for (const auto& subtask_pair : method.subtask.map_actions)
            {
                std::string formatted_subtask = formatSubtask(subtask_pair.second);
                method_msg.decomposition.push_back(formatted_subtask);
            }

            msg.methods.push_back(method_msg);
        }

        // Convert goals to effects
        for (const auto& goal : goals)
        {
            procedural_interfaces::TaskEffect effect_msg;
            effect_msg.subject = goal.subject;
            effect_msg.predicate = goal.property;
            effect_msg.object = goal.object;
            effect_msg.is_add = goal.add;
            msg.effects.push_back(effect_msg);
        }

        msg.is_primitive = methods_.empty();
        msg.description = "";

        return msg;
    }

private:
    std::string formatSubtask(const Ordered_Action_t& subtask) const
    {
        std::string formatted = subtask.name + "(";
        for (size_t i = 0; i < subtask.arguments.size(); ++i)
        {
            if (i > 0) formatted += ", ";
            formatted += subtask.arguments[i];
        }
        formatted += ")";
        return formatted;
    }

};

struct PrimitiveActionParsed_t {
    std::string name;
    std::vector<Arguments_t> arguments;
    std::vector<Expression_t> preconditions;
    effects_t effects;

    friend std::ostream& operator<<(std::ostream& os, const PrimitiveActionParsed_t& lhs)
    {
        os << "Action : " << lhs.name << "\n";
        for (const auto& arg: lhs.arguments)
            os << "\t" << arg << "\n";
        for (const auto& precondition: lhs.preconditions)
            os << "\t" << precondition << "\n";
        os << "\t" << lhs.effects;
        return os;

    }
};

struct HTNParserd_t {

    std::vector<Abstract_task_t> tasks;
    std::vector<PrimitiveActionParsed_t> actions;

    friend std::ostream& operator<<(std::ostream& os, const HTNParserd_t& lhs)
    {

        os << "ACTION : ----------------------------------------" << "\n";
        for (const auto& action: lhs.actions)
            os << action << "\n";
        os << "METHOD : ----------------------------------------" << "\n";
        for (const auto& method: lhs.tasks)
            os << method << "\n";
        return os;
    }

    bool empty() const
    {
        return tasks.empty() && actions.empty();
    }

};
}
#endif //PROCEDURAL_PARSEDHTN_H
