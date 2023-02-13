#include "procedural/core/Graph/Transition.h"
#include <iostream>

namespace procedural {

Transition::Transition(const FactPattern& pattern) : var_subject_(nullptr),
                                                     var_object_(nullptr)
{
    var_subject_str_ = pattern.getVarSubject();
    var_object_str_ = pattern.getVarObject();
    properties_.insert(pattern.getProperty());
}

void Transition::expandProperty()
{
    // TODO use ontology to expand the property
}

void Transition::linkVariables(std::map<std::string,Variable_t>& variables)
{
    var_object_ = &(variables.at(var_object_str_));
    var_subject_ = &(variables.at(var_subject_str_));
}

bool Transition::operator==(const Transition& other) const
{
    return (var_subject_str_ == other.var_subject_str_) &&
           (var_object_str_ == other.var_object_str_) &&
           (properties_ == other.properties_);
}

bool Transition::matchFact(const Fact& fact)
{
    bool match = (properties_.find(fact.getProperty()) != properties_.end());

    if(match)
    {
        if (var_subject_->getValue())
        {
            match = match && (fact.getSubject() == var_subject_->getValue());
            if(match)
            {
                if (var_object_->getValue())
                    match = match && (fact.getObject() == var_object_->getValue());
                else
                    setObject(fact.getObject());
            }
        }
        else
        {
            if (var_object_->getValue())
                match = match && (fact.getObject() == var_object_->getValue());
            else
                setObject(fact.getObject());

            if(match)
                setSubject(fact.getSubject());
        }       
    }
    
    return match;
}

std::string Transition::toString() const
{
    std::string str = var_subject_->toString() + " - (";
    for (auto property: properties_)
        str += " " + std::to_string(property);
    str += ") - " + var_object_->toString();

    return str;
}

} // namespace procedural