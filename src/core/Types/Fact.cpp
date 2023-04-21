#include "procedural/core/Types/Fact.h"

#include <iostream>

namespace procedural {

WordTable Fact::properties_table;
WordTable Fact::individuals_table;

Fact::Fact(bool add, const std::string& subject, const std::string& property, const std::string& object, uint32_t id,
           const TimeStamp_t& timestamp)
        : add_(add), id_(id), timestamp_(timestamp)
{
    property_ = properties_table.getConst(property);
    subject_ = individuals_table.get(subject);
    object_ = individuals_table.get(object);
}

bool Fact::operator==(const Fact& other) const
{
    return (add_ == other.add_) && (property_ == other.property_) &&
           (subject_ == other.subject_) && (object_ == other.object_);
}

std::string Fact::toString() const
{
    if (this->isValid())
        return ((this->add_) ? "ADD " : "DEL ") + individuals_table[this->subject_] + " " +
               properties_table[this->property_] + " " +
               individuals_table[this->object_];
    else
        return "";
}

} // namespace procedural