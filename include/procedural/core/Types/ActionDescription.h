#ifndef PROCEDURAL_ACTIONDESCRIPTION_H
#define PROCEDURAL_ACTIONDESCRIPTION_H

#include <map>
#include "procedural/core/Types/Variable.h"

namespace procedural
{
struct ActionDescription_t
{
    ActionDescription_t(std::string subject,std::string property,std::string object):subject_is_var_(false),object_is_var_(false)
    {
        subject_str_ = subject;
        object_str_ = object;
        property_ = property;
        verifVar();
    }
    
    void verifVar()
    {
        if(subject_str_[0] == '?')
        {
            if(subject_str_.length()==2 && subject_str_[1]=='?')
            {
                subject_str_ = "self";
                // subject_is_var_ = true;
            }
            else
            {
                subject_is_var_ = true;
                subject_str_ = subject_str_.substr(1);
            }
            
        }
        if(object_str_[0] == '?')
        {
            if(object_str_.length()==2 && object_str_[1]=='?')
                object_str_ = "self";
            else
            {
                object_is_var_ = true;
                object_str_ = object_str_.substr(1);
            }
        }

    }


    std::string toString()
    {
        std::string res = "[Description Action]";
        res += subject_str_+"|";
        res += property_+"|";
        res += object_str_;
        return res;
        
    }

    std::string subject_str_ ;
    std::string object_str_;

    bool subject_is_var_;
    bool object_is_var_;


    std::string property_;    
};
    
} // namespace procedural

#endif // PROCEDURAL_ACTIONDESCRIPTION_H