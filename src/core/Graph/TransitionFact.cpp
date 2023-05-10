#include "procedural/core/Graph/TransitionFact.h"
#include <iostream>
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>

namespace procedural {

TransitionFact::TransitionFact(const PatternFact& pattern) : var_subject_(nullptr),
                                                             var_object_(nullptr)
{
    var_subject_str_ = pattern.getVarSubject();
    var_object_str_ = pattern.getVarObject();
    str_initial_property = pattern.getStringProperty();
    properties_.insert(pattern.getProperty());
    insertion_ = (*properties_.begin())>0 ? true : false;
}

void TransitionFact::expandProperty(ObjectPropertyClient* object_client)
{

    auto res = object_client->getDown(str_initial_property);
    if (res.size()==1)
        return;
    std::cout << "expand property :" << str_initial_property << std::endl;
    for (const auto& result: res)
    {
        std::cout << "\t " << result << std::endl;
        auto property_id = Fact::properties_table.get(result);
        properties_.insert(insertion_? int(1*property_id) : int(-1*property_id));
    }

}

void TransitionFact::linkVariables(std::map<std::string, Variable_t>& variables)
{
    var_object_ = &(variables.at(var_object_str_));
    var_subject_ = &(variables.at(var_subject_str_));
}

bool TransitionFact::operator==(const TransitionFact& other) const
{
    return (var_subject_str_ == other.var_subject_str_) &&
           (var_object_str_ == other.var_object_str_) &&
           (properties_ == other.properties_);
}

bool TransitionFact::match(Fact* fact)
{
    bool match = (properties_.find(fact->getProperty()) != properties_.end());

    if (match)
    {
        if (var_subject_->getValue())
        {
            match = match && (fact->getSubject() == var_subject_->getValue());
            if (match)
            {
                if (var_object_->getValue())
                    match = match && (fact->getObject() == var_object_->getValue());
                else
                    setObject(fact->getObject());
            }
        } else
        {
            if (var_object_->getValue())
                match = match && (fact->getObject() == var_object_->getValue());
            else
                setObject(fact->getObject());

            if (match)
                setSubject(fact->getSubject());
        }
    }

    return match;
}

std::string TransitionFact::toString() const
{
    std::string str = var_subject_->toString() + " - (";
    for (auto property: properties_)
        str += " " + std::to_string(property);
    str += ") - " + var_object_->toString();

    return str;
}

} // namespace procedural