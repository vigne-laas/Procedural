#include "procedural/core/Types/FactPattern.h"

#include "procedural/core/Types/Fact.h"

namespace procedural {

FactPattern::FactPattern(bool is_insertion,
                         const std::string& varSubject,
                         const std::string& property,
                         const std::string& varObject,
                         bool required) :
        is_insertion_(is_insertion),
        required_(required),
        var_object_(varObject),
        var_subject_(varSubject)
{
    //TODO link with getOn/getUp Ontologenius pour hierarchie des proprietes
    property_ = Fact::properties_table.get(property);
}

std::string FactPattern::toString() const
{
    return "[" + std::string(is_insertion_ ? "ADD" : "DEL") + "] " +
           var_subject_ + " - " +
           getStringProperty() + " - " +
           var_object_ +
           (required_ ? " REQUIRED" : "");
}

} // namespace procedural