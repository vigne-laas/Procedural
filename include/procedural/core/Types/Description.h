#ifndef PROCEDURAL_DESCRIPTION_H
#define PROCEDURAL_DESCRIPTION_H

#include <map>
#include "procedural/core/Types/Variable.h"
#include "procedural/core/Types/ActionDescription.h"
#include "procedural/core/Graph/StateMachine.h"
#include "procedural/core/Types/Fact.h"

namespace procedural {

struct Description_t
{
    Description_t(const ActionDescription_t& des, std::map<std::string, Variable_t>& variables) : var_subject_str_(des.subject_),
                                                                                                  var_object_str_(des.object_),
                                                                                                  var_subject_(nullptr),
                                                                                                  var_object_(nullptr),
                                                                                                  property_(des.property_)
    {
        if (des.subject_is_var_)
        {
            if (variables.find(des.subject_) != variables.end())
                var_subject_ = &(variables.at(des.subject_));
            // else
            // throw NetworkException("Variable "+des.subject_+ " not found invalid description");
        }
        if (des.object_is_var_)
        {
            if (variables.find(des.object_) != variables.end())
                var_object_ = &(variables.at(des.object_));
            // else
            // throw NetworkException("Variable "+des.object_+ " not found invalid description");
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const Description_t& description)
    {
        os << description.explainExplicit();
        return os;
    }


    void linkVariables(std::map<std::string, Variable_t>& variables)
    {
        if (var_object_ != nullptr)
            var_object_ = &(variables.at(var_object_str_));
        if (var_subject_ != nullptr)
            var_subject_ = &(variables.at(var_subject_str_));
    }

    std::string explainExplicit() const
    {
        std::string res = "[ADD]";
        if (var_subject_ != nullptr)
            if (var_subject_->getValue() != 0)
                res += Fact::individuals_table[var_subject_->getValue()] + "|";
            else
                res += var_subject_->toString() + "|";
        else
            res += var_subject_str_ + "|";
        res += property_ + "|";
        if (var_object_ != nullptr)
            res += Fact::individuals_table[var_object_->getValue()];
        else
            res += var_object_str_;
        return res;

    }

    std::string explain() const
    {
        std::string res = "[ADD]";
        if (var_subject_ != nullptr)
            res += var_subject_->toString() + "|";
        else
            res += var_subject_str_ + "|";
        res += property_ + "|";
        if (var_object_ != nullptr)
            res += var_object_->toString();
        else
            res += var_object_str_;
        return res;

    }

    std::string var_subject_str_;
    std::string var_object_str_;

    Variable_t* var_subject_;
    Variable_t* var_object_;

    std::string property_;
};

} // namespace procedural

#endif // PROCEDURAL_DESCRIPTION_H