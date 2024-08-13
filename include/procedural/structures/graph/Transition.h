#ifndef PROCEDURAL_TRANSITION_H
#define PROCEDURAL_TRANSITION_H

#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>
#include "procedural/structures/Observation.h"

namespace procedural {
class node;
class Action;

class Transition {
public:
    Transition(uint64_t source_id, Observation* observation, uint64_t target_id, uint64_t id) :
            observation_(observation),
            source_id_(source_id),
            target_id_(target_id),
            id_(id) {
//        LOG_DEBUG << "Creating transition " << id_ << " from " << source_id_ << " to " << target_id_;
//        LOG_DEBUG << "Observation: " << observation_->toString();
    };

//    Transition(const Transition& other);

    void expandProperty(onto::ObjectPropertyClient* object_property_client);

    void linkVariables(std::map<std::string, std::shared_ptr<Variable_t>>& variables);

    uint64_t match(const Observation* observation) const;

    uint64_t getId() const { return id_; }
    uint64_t getSourceId() const { return source_id_; }
    uint64_t getTargetId() const { return target_id_; }
    Observation* getObservation() const { return observation_; }
    VariableTable_t getTableVariables() const { return observation_->table_variables_; }
    int getHtnId() const { return observation_->getHTNId(); }

    std::string toDot() const;
    std::string toString() const;
    friend std::ostream& operator<<(std::ostream& os, const Transition& transition);
    virtual ~Transition() {}
    void completeRemap(const std::vector<std::shared_ptr<Action>>& actions);


private:
    Observation* observation_;
    uint64_t id_;
    uint64_t source_id_;
    uint64_t target_id_;


};

} // procedural

#endif //PROCEDURAL_TRANSITION_H
