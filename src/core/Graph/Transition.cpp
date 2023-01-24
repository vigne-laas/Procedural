#include "procedural/core/Graph/Transition.h"
#include <iostream>

namespace procedural {

Transition::Transition(const FactPattern& pattern) : subject_(-1), object_(-1)
{
    var_subject_ = pattern.getVarSubject();
    var_object_ = pattern.getVarObject();
    properties_.insert(pattern.getProperty());
}

State* Transition::evolve(const Fact& fact) const
{
    if (matchFact(fact))
        return nextState_;
    else
        return nullptr;
}

bool Transition::matchFact(const Fact& fact) const
{
    bool match = true;
    match &= ((subject_ != -1) || (fact.getSubject() == subject_));
    match &= ((object_ != -1) || (fact.getObject() == object_));

    if(match)
        match &= (properties_.find(fact.getProperty()) != properties_.end());

    return match;
}

void Transition::expandProperty()
{
    // TODO use ontology to expand the property
}

bool Transition::operator==(const Transition& other) const
{
    return (subject_ == other.subject_) &&
           (object_ == other.object_) &&
           (nextState_ == other.nextState_) &&
           (properties_ == other.properties_);
}

std::string Transition::toString() const
{
    std::string str = ((subject_ != -1) ? std::to_string(subject_) : var_subject_) + " - (";
    for (auto property: properties_)
        str += " " + std::to_string(property);
    str += ") - " + ((object_ != -1) ? std::to_string(object_) : var_object_);

    return str;
}

void Transition::checkUpdate(const Network* pNetwork)
{
    subject_ = pNetwork->variables_map_.at(var_subject_);
    object_ = pNetwork->variables_map_.at(var_object_);
}


} // procedural