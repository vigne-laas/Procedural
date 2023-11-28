#ifndef PROCEDURAL_ACTIONDESCRIPTION_H
#define PROCEDURAL_ACTIONDESCRIPTION_H

#include <map>
#include "procedural/core/Types/Variable.h"

namespace procedural {
struct ActionDescription_t
{
    ActionDescription_t(const std::string& subject,
                        const std::string& property,
                        const std::string& object) : subject_(subject),
                                                     property_(property),
                                                     object_(object),
                                                     subject_is_var_(false),
                                                     object_is_var_(false)
    {
        verifyVar();
    }

    void verifyVar()
    {
        if (subject_[0] == '?')
        {
            if (subject_.length() == 2 && subject_[1] == '?')
            {
                subject_ = "self";
                // subject_is_var_ = true;
            }
            else
            {
                subject_is_var_ = true;
                subject_ = subject_.substr(1);
            }

        }
        if (object_[0] == '?')
        {
            if (object_.length() == 2 && object_[1] == '?')
                object_ = "self";
            else
            {
                object_is_var_ = true;
                object_ = object_.substr(1);
            }
        }
    }

    std::string toString()
    {
        std::string res = "[Description Action]";
        res += subject_ + "|";
        res += property_ + "|";
        res += object_;
        return res;
    }

    std::string subject_;
    std::string property_;
    std::string object_;

    bool subject_is_var_;
    bool object_is_var_;
};

} // namespace procedural

#endif // PROCEDURAL_ACTIONDESCRIPTION_H