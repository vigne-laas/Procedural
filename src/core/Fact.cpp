#include "procedural/core/Fact.h"

#include <iostream>


namespace procedural {

WordTable Fact::properties_table;
WordTable Fact::individuals_table;

Fact::Fact(bool add, const std::string& subject, const std::string& property, const std::string& object) : add_(add)
{
    property_ = properties_table.get_const(property);
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
        return (this->add_) ? "ADD " : "DEL " + individuals_table[this->subject_] + " " +
                                       properties_table[this->property_] + " " +
                                       individuals_table[this->object_];
    else
        return "";
}

} // procedural