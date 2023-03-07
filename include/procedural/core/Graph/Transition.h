#ifndef PROCEDURAL_TRANSITION_H
#define PROCEDURAL_TRANSITION_H

#include <cstdint> //FIXME Obliger ajouter ca ???
#include <unordered_set>

#include "procedural/core/Types/FactPattern.h"
#include "procedural/core/Types/Variable.h"

namespace procedural {

class Transition
{
public:
    explicit Transition(const FactPattern& pattern);

    void setSubject(int32_t subject) { var_subject_->value = subject; }
    void setObject(int32_t object) { var_object_->value = object; }

    const std::string& getVarSubject() const { return var_subject_->literal; }
    const std::string& getVarObject() const { return var_object_->literal; }

    void expandProperty();

    void linkVariables(std::map<std::string,Variable_t>& variables);

    bool operator==(const Transition& other) const;
    bool matchFact(Fact* fact);

    std::string toString() const;

private:

    std::string var_subject_str_ ;
    std::string var_object_str_;
    
    Variable_t * var_subject_;
    Variable_t * var_object_;

    std::unordered_set<int32_t> properties_;
};

} // namespace procedural

#endif // PROCEDURAL_TRANSITION_H
