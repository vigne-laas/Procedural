#include "procedural/core/Graph/Transition.h"
#include <iostream>

namespace procedural {

Transition::Transition(const FactPattern& pattern)
        : subject_(nullptr), object_(nullptr)
{
    var_subject_ = pattern.getVarSubject();
    var_object_ = pattern.getVarObject();
    properties_.insert(pattern.getProperty());
}


bool Transition::matchFact(const Fact& fact) const
{
    bool match = true;
//    match &= ((subject_->asValue()) || (fact.getSubject() == subject_->value));
//    match &= ((object_->asValue()) || (fact.getObject() == object_->value));
    if (subject_->asValue())
        match = match && (fact.getSubject() == subject_->value);
    if (object_->asValue())
        match = match && (fact.getObject() == object_->value);

    std::cout << "match :" << match << std::endl;

    if (match)
        match &= (properties_.find(fact.getProperty()) != properties_.end());

    return match;
}

void Transition::expandProperty()
{
    // TODO use ontology to expand the property
}

void Transition::linkVariables(std::vector<Variable_t>& variables)
{

    for (auto& variable: variables)
    {
        if (variable.literal == var_object_)
            object_ = &variable;
        if (variable.literal == var_subject_)
            subject_ = &variable;
    }
}

bool Transition::operator==(const Transition& other) const
{
    return (subject_ == other.subject_) &&
           (object_ == other.object_) &&
           (properties_ == other.properties_);
//           (nextState_ == other.nextState_) &&
//           && (properties_ == other.properties_);
}

std::string Transition::toString() const
{
    std::string str = ((subject_->asValue()) ? std::to_string(subject_->value) : var_subject_) + " - (";
    for (auto property: properties_)
        str += " " + std::to_string(property);
    str += ") - " + ((object_->asValue()) ? std::to_string(object_->value) : var_object_);

    return str;
}

//void Transition::checkUpdate(const Network* pNetwork)
//{
//    subject_ = pNetwork->variables_map_.at(var_subject_);
//    object_ = pNetwork->variables_map_.at(var_object_);
//}


} // procedural