#ifndef PROCEDURAL_TRANSITION_H
#define PROCEDURAL_TRANSITION_H

#include <cstdint> //FIXME Obliger ajouter ca ???
#include <unordered_set>

#include "procedural/core/Types/FactPattern.h"
#include "procedural/core/Graph/State.h"
#include "procedural/core/Types/Variable.h"

namespace procedural {

class State;

class Network;

class Transition
{
    friend Network;
public:
    explicit Transition(const FactPattern& pattern);

//    State* evolve(const Fact& fact) const;

    void setObject(int32_t object);

    void setSubject(int32_t subject);

    const std::string& getVarObject() const
    { return var_object_; }

    const std::string& getVarSubject() const
    { return var_subject_; }

    void expandProperty();

    void linkVariables(std::vector<Variable_t>& variables);

//    void setNextState(State * nextState) { nextState_ = nextState; }

    std::string toString() const;

    bool operator==(const Transition& other) const;

    bool matchFact(const Fact& fact) const;
    Variable_t * subject_;
    Variable_t * object_;

//    void checkUpdate(const Network* pNetwork);

private:
    std::string var_subject_;
    std::string var_object_;



//    State* nextState_;
    std::unordered_set<int32_t> properties_;
};

} // namespace procedural

#endif //PROCEDURAL_TRANSITION_H
