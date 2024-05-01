#include <algorithm>
#include "procedural/structures/Fact.h"
#include "procedural/utils/Logger.h"
#include "procedural/structures/VariablesTable.h"

namespace procedural {
uint32_t Fact::id_fact = 0;

Fact::Fact(bool add, const std::string& subject, const std::string& subject_type, const std::string& property,
           const std::string& object, const std::string& object_type, const TimeStamp_t& time) : add_(add),
                                                                                                 timestamp_(time),
                                                                                                 id_(++id_fact)
{
//    LOG_DEBUG << "Fact constructor\n";
//    LOG_DEBUG << "subject: " << subject << " property: " << property << " object: " << object << "\n";
    subject_ = std::make_pair(subject, std::make_shared<Variable_t>(subject_type));
    object_ = std::make_pair(object, std::make_shared<Variable_t>(object_type));
    id_property_ = WordTable::properties_table.getConst(property);
//    LOG_DEBUG << "properties : " << WordTable::properties_table.toString() << "\n";
//    LOG_DEBUG << "id_property_ : " << id_property_ << "\n";
    id_extended_properties_.insert(add_ ? int32_t(1 * id_property_) : int32_t(-1 * id_property_));
    subject_.second->value_ = WordTable::individuals_table.get(subject);
    object_.second->value_ = WordTable::individuals_table.get(object);
}

Fact::Fact(bool add, const std::string& literal_subject, const std::string& subject_type, const std::string& property,
           const std::string& literal_object, const std::string& object_type) : add_(add), id_(0)
{
    subject_ = std::make_pair(literal_subject, std::make_shared<Variable_t>(subject_type));
    object_ = std::make_pair(literal_object, std::make_shared<Variable_t>(object_type));
    id_property_ = WordTable::properties_table.getConst(property);
}


std::string Fact::toString() const
{
    std::string result;

    if (id_ != 0) {
        result += "[" + std::to_string(id_) + "] ";
    }

    result += (this->add_) ? "ADD " : "DEL ";

    result += subject_.second->toString(subject_.first);
    result += "[" + std::to_string(reinterpret_cast<uintptr_t>(subject_.second.get())) + "]";
    result += "|" + WordTable::properties_table[id_property_] + "|";
    result += object_.second->toString(object_.first);
    result += "[" + std::to_string(reinterpret_cast<uintptr_t>(object_.second.get())) + "]";
    if (!this->id_extended_properties_.empty()) {
        result += " extended properties : " + std::to_string(id_extended_properties_.size()) + " ";
    }

    result += "\n";

    return result;
}

std::string Fact::toShortString() const
{
    return ((this->add_) ? "+ " : "- ") + std::to_string(this->subject_.second->getValue()) + "|" +
           std::to_string(this->id_property_) + "|" + std::to_string(this->object_.second->getValue());
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
//        LOG_DEBUG << "intersection size : " << intersection.size() << "\n";
//        LOG_DEBUG << "intersection : \n";
//        for (const auto& id: intersection) {
//            LOG_DEBUG << "\t" << id << "\n";
//        }
        if (intersection.empty()) {
            LOG_DEBUG << "intersection is empty\n";
            return false;
        }
    }


//    LOG_DEBUG << "id_subject_ : " << this->subject_->toString() << " other.id_subject_ : " << other.subject_->toString()
//              << "\n";
//    LOG_DEBUG << "id_object_ : " << this->id_object_ << " other.id_object_ : " << other.id_object_ << "\n";
    if (*subject_.second != *other.subject_.second) {
        LOG_DEBUG << "subject is diff\n";
        return false;
    }

    if (*object_.second != *other.object_.second) {
        LOG_DEBUG << "object is diff\n";
        return false;
    }


    return true;
}

void Fact::expandProperty(onto::ObjectPropertyClient* object_property_client)
{

    if (id_property_ == 0) {
        LOG_ERROR << "expandProperty : id_property_ is 0\n";
        return;
    }

    LOG_INFO << "expandProperty from : " << WordTable::properties_table[id_property_] << "\n";
    auto extended_properties = object_property_client->getDown(WordTable::properties_table[id_property_]);
//    LOG_INFO << "expandProperty " << WordTable::properties_table[id_property_] << " : " << extended_properties.size() << "\n";
//    for (const auto& property: extended_properties) {
//        LOG_INFO << "\t - add extended property :" << property << "\n";
//    }
    if (extended_properties.size() == 1)
        return;
    for (const auto& property: extended_properties) {
        LOG_INFO << "\t - add extended property :" << property << "\n";
        auto id = WordTable::properties_table.get(property);
        id_extended_properties_.insert(add_ ? int32_t(1 * id) : int32_t(-1 * id));
    }
//    LOG_INFO << "extended properties : " << id_extended_properties_.size() << "\n";
//    for (const auto& id: id_extended_properties_) {
//        LOG_INFO << "\t - extended property :" << WordTable::properties_table[std::abs(id)] << "\n";
//    }

}

void Fact::link(VariableTable_t& table_variables)
{
//    LOG_DEBUG << "Before Fact::link : " << this->toString() << "\n";
    auto old_subject = subject_;
    subject_ = std::make_pair(subject_.first, table_variables.variables[subject_.first]);
    if (old_subject.second->isSet())
        subject_.second->value_ = old_subject.second->value_;

    auto old_object = object_;
    object_ = std::make_pair(object_.first, table_variables.variables[object_.first]);
    if (old_object.second->isSet())
        object_.second->value_ = old_object.second->value_;

//    LOG_DEBUG << "After Fact::link : " << this->toString() << "\n";

//    object_->value_ = individuals_table.get(object_->literal_);
//    LOG_DEBUG << "subject is set ? " << subject_->isSet() << " object is set ? " << object_->isSet() << "\n";
}

Fact::Fact(const Fact& other) : id_(0)
{
    add_ = other.add_;
    subject_ = std::make_pair(other.subject_.first, std::make_shared<Variable_t>(*other.subject_.second));
//    subject_->value_ = other.subject_->value_;
    object_ = std::make_pair(other.object_.first, std::make_shared<Variable_t>(*other.object_.second));
//    object_->value_ = other.object_->value_;
    id_property_ = other.id_property_;
    id_extended_properties_ = other.id_extended_properties_;
    timestamp_ = other.timestamp_;

}

Fact::Fact(ParsedFact_t& parsed_fact) : id_(0)
{
    add_ = parsed_fact.insertion;
    subject_ = std::make_pair(parsed_fact.subject, std::make_shared<Variable_t>(parsed_fact.subject_type));
    object_ = std::make_pair(parsed_fact.object, std::make_shared<Variable_t>(parsed_fact.object_type));
    id_property_ = WordTable::properties_table.getConst(parsed_fact.property);
//    id_extended_properties_.insert(add_ ? int32_t(1 * id_property_) : int32_t(-1 * id_property_));
//    subject_->value_ = WordTable::individuals_table.get(parsed_fact.subject);
//    object_->value_ = WordTable::individuals_table.get(parsed_fact.object);
}

Fact::Fact(const ParsedFact_t& parsed_fact) : id_(0)
{
    add_ = parsed_fact.insertion;
    subject_ = std::make_pair(parsed_fact.subject, std::make_shared<Variable_t>(parsed_fact.subject_type));
    object_ = std::make_pair(parsed_fact.object, std::make_shared<Variable_t>(parsed_fact.object_type));
    id_property_ = WordTable::properties_table.getConst(parsed_fact.property);
//    id_extended_properties_.insert(add_ ? int32_t(1 * id_property_) : int32_t(-1 * id_property_));


}

Fact::Fact(ParsedFact_t& parsed_fact, const std::map<std::string, std::string>& type_map)
{

    add_ = parsed_fact.insertion;
    subject_ = std::make_pair(parsed_fact.subject, std::make_shared<Variable_t>(type_map.at(parsed_fact.subject)));
    object_ = std::make_pair(parsed_fact.object, std::make_shared<Variable_t>(type_map.at(parsed_fact.object)));
    id_property_ = WordTable::properties_table.getConst(parsed_fact.property);
//    id_extended_properties_.insert(add_ ? int32_t(1 * id_property_) : int32_t(-1 * id_property_));

}

Fact::Fact(const ParsedFact_t& parsed_fact, const std::map<std::string, std::string>& type_map)
{
    add_ = parsed_fact.insertion;
    subject_ = std::make_pair(parsed_fact.subject, std::make_shared<Variable_t>(type_map.at(parsed_fact.subject)));
    object_ = std::make_pair(parsed_fact.object, std::make_shared<Variable_t>(type_map.at(parsed_fact.object)));
    id_property_ = WordTable::properties_table.getConst(parsed_fact.property);
//    id_extended_properties_.insert(add_ ? int32_t(1 * id_property_) : int32_t(-1 * id_property_));
}


} // procedural