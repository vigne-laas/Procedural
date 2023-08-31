#ifndef PROCEDURAL_TRANSITIONFACT_H
#define PROCEDURAL_TRANSITIONFACT_H

#include <unordered_set>
#include "procedural/core/Types/PatternFact.h"
#include "procedural/core/Types/Variable.h"
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>

namespace procedural {

class TransitionFact
{
public:
    explicit TransitionFact(const PatternFact& pattern);

    void setSubject(int32_t subject) { var_subject_->value = subject; }
    void setObject(int32_t object) { var_object_->value = object; }

    const std::string& getVarSubject() const { return var_subject_->literal; }
    const std::string& getVarObject() const { return var_object_->literal; }

    void expandProperty(onto::ObjectPropertyClient* object_client);

    void linkVariables(std::map<std::string, Variable_t>& variables);

    bool operator==(const TransitionFact& other) const;
    bool match(Fact* fact) ;
    std::string toString() const;

private:

    std::string var_subject_str_;
    std::string var_object_str_;

    Variable_t* var_subject_;
    Variable_t* var_object_;

    std::unordered_set<int32_t> properties_;
    std::string str_initial_property;
    bool insertion_;
};

} // namespace procedural

#endif // PROCEDURAL_TRANSITIONFACT_H
