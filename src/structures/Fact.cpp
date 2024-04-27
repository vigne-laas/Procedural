#include <algorithm>
#include "procedural/structures/Fact.h"
#include "procedural/utils/Logger.h"
#include "procedural/structures/VariablesTable.h"

namespace procedural {

WordTable Fact::properties_table;
WordTable Fact::individuals_table;

Fact::Fact(bool add, const std::string& subject, const std::string& subject_type, const std::string& property,
           const std::string& object, const std::string& object_type, const TimeStamp_t& time) : add_(add),
                                                                                                 timestamp_(time),id_(0)
{
//    LOG_DEBUG << "Fact constructor\n";
//    LOG_DEBUG << "subject: " << subject << " property: " << property << " object: " << object << "\n";
    subject_ = std::make_shared<Variable_t>(subject, subject_type);
    object_ = std::make_shared<Variable_t>(object, object_type);
    id_property_ = properties_table.getConst(property);
    id_extended_properties_.insert(add_ ? int32_t(1 * id_property_) : int32_t(-1 * id_property_));
    subject_->value_ = individuals_table.get(subject);

    object_->value_ = individuals_table.get(object);
}

Fact::Fact(bool add, const std::string& literal_subject, const std::string& subject_type, const std::string& property,
           const std::string& literal_object, const std::string& object_type): add_(add),id_(0)
{
    subject_ = std::make_shared<Variable_t>(literal_subject, subject_type);
    object_ = std::make_shared<Variable_t>(literal_object, object_type);
    id_property_ = properties_table.getConst(property);
}


std::string Fact::toString() const
{
    return ((this->add_) ? "ADD " : "DEL ") + this->subject_->toString() +
           "|" + properties_table[id_property_] + "|" +
           this->object_->toString() +
           ((this->id_extended_properties_.empty()) ? "" : " extended properties : " +
                                                           std::to_string(this->id_extended_properties_.size()) + " ") +
           "\n";
}

std::string Fact::toShortString() const
{
    return ((this->add_) ? "+ " : "- ") + std::to_string(this->subject_->getValue()) + "|" +
           std::to_string(this->id_property_) + "|" + std::to_string(this->object_->getValue());
}

std::ostream& operator<<(std::ostream& os, const Fact& fact)
{
    os << fact.toString();
    return os;
}

bool Fact::match(const procedural::Fact& other) const
{
//    LOG_DEBUG << this->toString() << " =? " << other.toString() << "\n";

    if ((this->add_ != other.add_))
        return false;
    if (this->id_property_ != other.id_property_) {
        if (this->id_extended_properties_.size() == 1 and other.id_extended_properties_.size() == 1)
            return false;
        std::unordered_set<uint32_t> intersection;
        std::set_intersection(this->id_extended_properties_.begin(), this->id_extended_properties_.end(),
                              other.id_extended_properties_.begin(), other.id_extended_properties_.end(),
                              std::inserter(intersection, intersection.begin()));
        LOG_DEBUG << "intersection size : " << intersection.size() << "\n";
        LOG_DEBUG << "intersection : \n";
        for (const auto& id: intersection) {
            LOG_DEBUG << "\t" << id << "\n";
        }
        if (intersection.empty()) {
            LOG_DEBUG << "intersection is empty\n";
            return false;
        }
    }


//    LOG_DEBUG << "id_subject_ : " << this->subject_->toString() << " other.id_subject_ : " << other.subject_->toString()
//              << "\n";
//    LOG_DEBUG << "id_object_ : " << this->id_object_ << " other.id_object_ : " << other.id_object_ << "\n";
    if (subject_ != other.subject_)
        return false;
    if (object_ != other.object_)
        return false;

//    if (this->subject_->isSet() and this->subject_->getValue() != other.subject_->getValue()) {
////        LOG_DEBUG << "subject is set and diff on value\n";
//        return false;
//    }
//
//    if (this->object_->isSet() and this->object_->getValue() != other.object_->getValue()) {
////        LOG_DEBUG << "object is set and diff on value\n";
//        return false;
//    }


    return true;
}

void Fact::expandProperty(onto::ObjectPropertyClient* object_property_client)
{
    if (id_property_ == 0)
        return;
//    LOG_INFO << "expandProperty " << properties_table[id_property_] << "\n";
    auto extended_properties = object_property_client->getDown(properties_table[id_property_]);
    LOG_INFO << "expandProperty " << properties_table[id_property_] << " : " << extended_properties.size() << "\n";
    for (const auto& property: extended_properties) {
        LOG_INFO << "\t - add extended property :" << property << "\n";
    }
    if (extended_properties.size() == 1)
        return;
    for (const auto& property: extended_properties) {
//        LOG_INFO << "\t - add extended property :" << property << "\n";
        auto id = properties_table.get(property);
        id_extended_properties_.insert(add_ ? int32_t(1 * id) : int32_t(-1 * id));
    }

}

void Fact::link(VariableTable_t& table_variables)
{
//    LOG_DEBUG << "Fact::link : " << this->toString() << "\n";
    auto old_subject = subject_;
    subject_ = table_variables.variables[subject_->literal_];
    if (old_subject->isSet())
        subject_->value_ = old_subject->value_;
//    subject_->value_ = individuals_table.get(subject_->literal_);
    auto old_object = object_;
    object_ = table_variables.variables[object_->literal_];
    if (old_object->isSet())
        object_->value_ = old_object->value_;
//    object_->value_ = individuals_table.get(object_->literal_);
//    LOG_DEBUG << "subject is set ? " << subject_->isSet() << " object is set ? " << object_->isSet() << "\n";
}

Fact::Fact(const Fact& other): id_(0)
{
    add_ = other.add_;
    subject_ = std::make_shared<Variable_t>(*other.subject_);
//    subject_->value_ = other.subject_->value_;
    object_ = std::make_shared<Variable_t>(*other.object_);
//    object_->value_ = other.object_->value_;
    id_property_ = other.id_property_;
    id_extended_properties_ = other.id_extended_properties_;
    timestamp_ = other.timestamp_;

}

Fact::Fact(ParsedFact_t& parsed_fact): id_(0)
{
    add_ = parsed_fact.required;
    subject_ = std::make_shared<Variable_t>(parsed_fact.subject, parsed_fact.subject_type);
    object_ = std::make_shared<Variable_t>(parsed_fact.object, parsed_fact.object_type);
    id_property_ = properties_table.getConst(parsed_fact.property);
    id_extended_properties_.insert(add_ ? int32_t(1 * id_property_) : int32_t(-1 * id_property_));
    subject_->value_ = individuals_table.get(parsed_fact.subject);
    object_->value_ = individuals_table.get(parsed_fact.object);
}


} // procedural