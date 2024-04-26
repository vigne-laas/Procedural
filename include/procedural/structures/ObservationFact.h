#ifndef PROCEDURAL_OBSERVATIONFACT_H
#define PROCEDURAL_OBSERVATIONFACT_H

#include "procedural/structures/Observation.h"
#include "procedural/structures/Fact.h"

namespace procedural {
class ObservationFact : public Observation {
public:
    explicit ObservationFact(const Fact& fact);

    bool operator==(const Observation& other) override;

    void expandProperty(onto::ObjectPropertyClient* object_property_client) override;
    void linkVariables(std::map<std::string, std::shared_ptr<Variable_t>>& variables) override;

    void completeVar(const std::vector<std::shared_ptr<Action>>& actions) override {};

    std::string toString() const override;
    Fact getFact() const { return fact_; }

private:
    Fact fact_;
};
} // procedural
#endif //PROCEDURAL_OBSERVATIONFACT_H
