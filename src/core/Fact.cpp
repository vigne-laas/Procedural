#include "procedural/core/Fact.h"

#include <iostream>


namespace procedural {

WordTable Fact::table_properties_;
WordTable Fact::table_subjects_objects_;

/*WordTable table_properties = WordTable();
table_properties.add("isA");
table_properties.add("isInContainer");
table_properties.add("isOnTopOf");
WordTable table_objects = WordTable();*/

/*WordTable Fact::table_properties_ = WordTable();

WordTable Fact::table_subjects_objects_ = table_objects;*/



Fact::Fact(bool add, const std::string& subject, const std::string& property, const std::string& object) : add_(add)
{
    property_ = table_properties_.get_const(property);
    subject_ = table_subjects_objects_.get(subject);
    object_ = table_subjects_objects_.get(object);
}

bool Fact::isValid() const
{
    return property_ != 0;
}

int32_t Fact::getProperty() const
{
    return add_ ? property_ : property_ * -1;
}

uint32_t Fact::getSubject() const
{
    return subject_;
}

uint32_t Fact::getObject() const
{
    return object_;
}

bool Fact::operator==(const Fact& F2)
{
    return (this->add_ == F2.add_) and (this->property_ == F2.property_) and (this->subject_ == F2.subject_) and
           (this->object_ == F2.object_);
}

std::string Fact::toString() const
{
    std::cout << table_subjects_objects_[object_] << std::endl;
    if (this->isValid())
        return ((this->add_) ? "ADD " : "DEL " )+ table_subjects_objects_[this->subject_] + " " +
                                       table_properties_[this->property_] + " " +
                                       table_subjects_objects_[this->object_];
    else
        return "invalid Fact";
}

std::string Fact::getStringProperty() const
{
    return table_properties_[property_];
}

std::string Fact::getStringSubject() const
{
    return table_subjects_objects_[subject_];
}

std::string Fact::getStringObject() const
{
//    table_subjects_objects_.printAll();
    return table_subjects_objects_[object_];
}

void Fact::printProperties() const
{
    table_properties_.printAll();
}

void Fact::printSubjectsObjects() const
{
    table_subjects_objects_.printAll();
}

} // procedural