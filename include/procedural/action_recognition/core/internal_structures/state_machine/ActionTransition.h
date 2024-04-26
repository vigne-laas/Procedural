#ifndef PROCEDURAL_ACTIONTRANSITION_H
#define PROCEDURAL_ACTIONTRANSITION_H

#include <string>
#include <ontologenius/OntologyManipulator.h>
#include "procedural/action_recognition/core/internal_structures/state_machine/structures/ActionObservation.h"

namespace procedural {
enum class ActionTransitionType {
    UNSPECIFIED,
    FACT,
    ACTION
};

class IActionTransition {
public:
    virtual ~IActionTransition() = default;
    virtual bool match(ActionObservation_t* observation) = 0;
    virtual std::string toString() const = 0;
    virtual std::string toShortString() const = 0;
};


template <typename PatternType, ActionTransitionType transitionType>
class ActionTransition : public IActionTransition {
public:
    ActionTransition(const PatternType& pattern) : type_(transitionType), vars_(pattern) {}

    bool match(ActionObservation_t* observation) override {
        return vars_.match(observation);
    }

    std::string toString() const override {
        std::string msg = "Transition ";
        msg += (transitionType == ActionTransitionType::FACT) ? "Fact :" : "Action :";
        msg += vars_.toString();
        return msg;
    }

    std::string toShortString() const override {
        std::string msg = "Transition ";
        msg += (transitionType == ActionTransitionType::FACT) ? "Fact :" : "Action :";
        msg += vars_.toShortString();
        return msg;
    }

private:
    ActionTransitionType type_;
    TransitionVariable_t<PatternType> vars_;
};


};

}//action_recognition
#endif //PROCEDURAL_ACTIONTRANSITION_H
