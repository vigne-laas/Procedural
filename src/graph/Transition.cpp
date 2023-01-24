#include "procedural/graph/Transition.h"
#include <iostream>

namespace procedural {
Transition::Transition() : subject_(-1), object_(-1)
{

}

procedural::State* Transition::evolve(const Fact& fact) const
{

    if (*this == fact)
        return nextState_;
    else
        return nullptr;
}

bool Transition::operator==(const Fact& fact) const
{
//    std::cout << "try evolve subject : " << std::endl;
    bool condition = true;
    if (subject_ != -1)
        condition = condition && (fact.getSubject() == subject_);
    if (object_ != -1)
        condition = condition && (fact.getObject() == object_);
//    std::cout << "try evolve condition : "<< condition << std::endl;
    if (condition)
        if (properties_.find(fact.getProperty()) != properties_.end())
        {
//            std::cout << " == return true" << std::endl;
            return true;
        } else
        {
//            std::cout << " == return false" << std::endl;
            return false;
        }


    return false;
}

void Transition::setObject(int32_t object)
{
    object_ = object;
}

void Transition::setSubject(int32_t subject)
{
    subject_ = subject;
}


void Transition::addProperty(int32_t property)
{
    properties_.insert(property);
}

void Transition::setProperties(const std::unordered_set<int32_t>& properties)
{
    properties_ = properties;
}

bool Transition::operator==(const Transition& transition) const
{
    return (subject_ == transition.subject_) and (object_ == transition.object_) and
           (nextState_ == transition.nextState_) and (properties_ == transition.properties_);
}


void Transition::setNextState(State* nextState)
{
    nextState_ = nextState;
}

std::string Transition::toString() const
{
    std::string msg =
            "subject : " + (subject_ != -1 ? std::to_string(subject_) : var_subject_) + "\n\t\t -  properties : \n";
    for (auto property: properties_)
    {
        msg += "\t\t\t -  " + std::to_string(property) + "\n";
    }
    msg += "\t\t -  object : " + (object_ != -1 ? std::to_string(object_) : var_object_) + "\n";
    msg += "\t\t -  next State : " + nextState_->name_ + "_" + std::to_string(nextState_->id_) + "\n";

    return msg;


}

void Transition::setVarObject(const std::string& object)
{
    var_object_ = object;
}

void Transition::setVarSubject(const std::string& subject)
{
    var_subject_ = subject;
}

const std::string& Transition::getVarObject() const
{
    return var_object_;
}

const std::string& Transition::getVarSubject() const
{
    return var_subject_;
}

void Transition::checkUpdate(const Network* pNetwork)
{

    subject_ = pNetwork->variables_map_.at(var_subject_);
    object_ = pNetwork->variables_map_.at(var_object_);


}


} // procedural