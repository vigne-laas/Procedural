#ifndef PROCEDURAL_PARSEDHTN_H
#define PROCEDURAL_PARSEDHTN_H

#include <vector>
#include <string>
#include <iostream>
#include <set>
#include <map>

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

    friend std::ostream& operator<<(std::ostream& os, const Expression_t& lhs)
    {
        os << lhs.subject << " " << lhs.property << " " << lhs.object;
        return os;
    }
};


struct Arguments_t {
    Arguments_t(const std::string& type, const std::string& varname) : type(type), varname(varname) {};
    std::string type;
    std::string varname;

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
    std::vector<Expression_t> preconditions;
    Subtask_t subtask;

    friend std::ostream& operator<<(std::ostream& os, const Method_t& lhs)
    {
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
