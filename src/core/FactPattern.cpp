#include <iostream>
#include "procedural/core/FactPattern.h"
#include "procedural/core/Fact.h"

namespace procedural {
FactPattern::FactPattern(bool negative, const std::string& varSubject, const std::string& property,
                         const std::string& varObject, bool required) :
        negative_(negative), required_(required),
        var_object_(varObject), var_subject_(varSubject)
{
    //TODO link with getOn/getUp Ontologenius pour hierarchie des proprietes
    property_ = Fact::table_properties_.get(property);
//    std::cout<<"Constructeur Pattern"<< std::endl;
//    std::cout << "\t\t subject : " << var_subject_ << std::endl;
//    std::cout << "\t\t object : " << var_object_ << std::endl;
//    Fact::table_properties_.printAll();
}

int32_t FactPattern::getProperty() const
{
    return negative_ ? property_ * -1 : property_; //FIXME change value to no negative form
}

const std::string& FactPattern::getStringProperty() const
{
    return Fact::table_properties_[property_];
}

const std::string& FactPattern::getVarSubject() const
{
    return var_subject_;
}

const std::string& FactPattern::getVarObject() const
{
    return var_object_;
}

std::string FactPattern::to_string() const
{
    std::string msg = "Fact Pattern : \n\t";
    msg += negative_ ? "Negative property\n\t" : "";
    msg += required_ ? "Required\n\t" : "";
    msg+= "\t subject : "+ var_subject_+"\n";
    msg+= "\t property : "+ getStringProperty()+"\n";
    msg+= "\t object : "+ var_object_+"\n";

    return msg;
}

} // procedural