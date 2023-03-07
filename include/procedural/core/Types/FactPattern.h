#ifndef PROCEDURAL_FACTPATTERN_H
#define PROCEDURAL_FACTPATTERN_H

#include <string>

#include "procedural/core/Types/Fact.h"

namespace procedural {

class FactPattern
{

public:
    FactPattern(bool is_insertion, const std::string& var_subject, const std::string& property,
                const std::string& var_object, bool required);

    int32_t getProperty() const { return (int32_t)property_ * (is_insertion_ ?  1 : -1); } 
    const std::string& getStringProperty() const { return Fact::properties_table[property_]; };

    const std::string& getVarSubject() const { return var_subject_; }
    const std::string& getVarObject() const { return var_object_; }

    bool isRequired() const { return required_; }

    std::string toString() const;

private:
    const std::string var_subject_;
    const std::string var_object_;

    bool is_insertion_;
    bool required_;
    uint32_t property_;
};

} // namespace procedural

#endif // PROCEDURAL_FACTPATTERN_H
