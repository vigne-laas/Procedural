#ifndef PROCEDURAL_TRANSITION_H
#define PROCEDURAL_TRANSITION_H

#include <cstdint> //FIXME Obliger ajouter ca ???
#include <unordered_set>

#include "procedural/core/Types/FactPattern.h"
#include "procedural/core/Graph/State.h"
#include "procedural/graph/Network.h"

namespace procedural {

class State;
class Network;

class Transition
{
    friend Network;
public:
    explicit Transition(const FactPattern& pattern);

    State* evolve(const Fact& fact) const;

    void setObject(int32_t object);
    void setSubject(int32_t subject);

    const std::string& getVarObject() const { return var_object_; }
    const std::string& getVarSubject() const { return var_subject_; }

    void expandProperty();

    void setNextState(State * nextState) { nextState_ = nextState; }

    std::string toString()const;

    bool operator==(const Transition& other) const;
    bool matchFact(const Fact& fact) const;

    void checkUpdate(const Network* pNetwork);

private:
    std::string var_subject_;
    std::string var_object_;

    int32_t subject_;
    int32_t object_;

    State* nextState_;
    std::unordered_set<int32_t> properties_;
};

} // namespace procedural

#endif //PROCEDURAL_TRANSITION_H
