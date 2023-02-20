#ifndef PROCEDURAL_DESCRIPTION_H
#define PROCEDURAL_DESCRIPTION_H

#include <map>
#include "procedural/core/Types/Variable.h"

namespace procedural
{
    //FIXME make subject or object as optional variables if already set in the file 

struct Description_t
{
    Description_t(std::string subject,std::string property,std::string object)
    {
        var_subject_str_ = subject;
        var_object_str_ = object;
        property = property;
    }
 

    void linkVariables(std::map<std::string,Variable_t>& variables)
    {
        var_object_ = &(variables.at(var_object_str_));
        var_subject_ = &(variables.at(var_subject_str_));
    }

    std::string explain()
    {
        std::string res = "[ADD]";
        res += var_subject_->toString()+"|";
        res += property+"|";
        res += var_object_->toString();
        return res;
        
    }

    std::string var_subject_str_ ;
    std::string var_object_str_;
    
    Variable_t * var_subject_;
    Variable_t * var_object_; 

    std::string property;    
};
    
} // namespace procedural

#endif // PROCEDURAL_DESCRIPTION_H