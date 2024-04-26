#ifndef PROCEDURAL_OBSERVATION_H
#define PROCEDURAL_OBSERVATION_H

#include "procedural/structures/VariablesTable.h"
#include "procedural/utils/Logger.h"
#include <iostream>
#include <string>
#include <map>
#include <unordered_set>
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>

namespace procedural {
class Action;

class Observation {
public:
    explicit Observation(int64_t id) : id_(id) {}

    Observation(int64_t id, VariableTable_t& table_variables) : id_(id), table_variables_(table_variables) {}

    Observation(const Observation& other);

    virtual ~Observation() {}

    virtual std::pair<int64_t, VariableTable_t> getData()
    {
        return {id_, table_variables_};
    }


    virtual void linkVariables(std::map<std::string, std::shared_ptr<Variable_t>>& variables);

    virtual bool operator==(const Observation& other);

    int64_t getId() const { return id_; }

    virtual void expandProperty(onto::ObjectPropertyClient* object_property_client) {};

    virtual std::string toString() const;

    friend std::ostream& operator<<(std::ostream& os, const Observation& obs);


    VariableTable_t table_variables_;

    void setRemap(std::map<std::string, std::string> remap_param);

    virtual void completeVar(const std::vector<std::shared_ptr<Action>>& actions);

protected:
    int64_t id_;
    std::map<std::string, std::string> remap_;

};

} // procedural

#endif //PROCEDURAL_OBSERVATION_H
