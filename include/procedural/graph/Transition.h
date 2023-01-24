#ifndef PROCEDURAL_TRANSITION_H
#define PROCEDURAL_TRANSITION_H

#include <cstdint> //FIXME Obliger ajouter ca ???
#include <unordered_set>
#include "procedural/core/Types/Fact.h"
#include "procedural/graph/State.h"
#include "procedural/graph/Network.h"

namespace procedural {

class State;
class Network;

class Transition
{
    friend Network;
public:
    Transition();

    State* evolve(const Fact& fact) const;

    void setObject(int32_t object);
    void setSubject(int32_t subject);
    void setVarObject(const std::string& object);
    void setVarSubject(const std::string& subject);
    const std::string& getVarObject()const;
    const std::string& getVarSubject()const;
    void setProperties(const std::unordered_set<int32_t>& properties);
    void addProperty(int32_t property);


    void setNextState(State * nextState);

    std::string toString()const;



    bool operator==(const Transition& transition) const;
    bool operator==(const Fact& fact) const;

    void checkUpdate(const Network* pNetwork);

private:
    std::string var_subject_;
    std::string var_object_;
    int32_t subject_;
    int32_t object_;
    State* nextState_;
    std::unordered_set<int32_t> properties_;


};

} // procedural

#endif //PROCEDURAL_TRANSITION_H
