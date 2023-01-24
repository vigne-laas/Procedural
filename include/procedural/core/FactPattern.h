#ifndef PROCEDURAL_FACTPATTERN_H
#define PROCEDURAL_FACTPATTERN_H

#include <string>
#include "procedural/graph/Network.h"

namespace procedural {


class FactPattern
{

public:
    FactPattern(bool negative, const std::string& var_subject, const std::string& property,
                const std::string& var_object, bool required);

    int32_t getProperty() const;

    const std::string& getStringProperty() const;

    const std::string& getVarSubject() const;

    const std::string& getVarObject() const;

    bool isRequired() const
    {
        return required_;
    }

    std::string to_string() const;


private:
    const std::string var_subject_;
    const std::string var_object_;


    bool negative_;
    bool required_;
    uint32_t property_;


};

} // procedural

#endif //PROCEDURAL_FACTPATTERN_H
