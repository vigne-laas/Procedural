#ifndef PROCEDURAL_FACT_H
#define PROCEDURAL_FACT_H

#include <string>
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>
#include "procedural/utils/TimeStamp.h"
#include "procedural/utils/WordTable.h"
#include "procedural/structures/Variable.h"
#include "procedural/action_recognition/reader/types/subtypes/ParsedFacts.h"

namespace procedural {
struct VariableTable_t;

class Fact {
public:


    Fact(bool add, const std::string& subject, const std::string& subject_type, const std::string& property,
         const std::string& object, const std::string& object_type, const TimeStamp_t& time);

    Fact(ParsedFact_t& parsed_fact);

    Fact(ParsedFact_t& parsed_fact, const std::map<std::string, std::string>& type_map);

    Fact(const ParsedFact_t& parsed_fact);

    Fact(const ParsedFact_t& parsed_fact, const std::map<std::string, std::string>& type_map);

    Fact(const Fact& other);

    Fact(bool add, const std::string& literal_subject, const std::string& subject_type, const std::string& property,
         const std::string& literal_object, const std::string& object_type);

    std::string getLiteralSubject() const { return subject_.first; }

    std::string getLiteralObject() const { return object_.first; }

    std::string getStrSubject() const
    {
        return (subject_.second->isSet()) ? WordTable::individuals_table[subject_.second->getValue()] : subject_.first;
    }

    std::string getStrProperty() const { return WordTable::properties_table[id_property_]; }

    std::string getStrObject() const
    {
        return (object_.second->isSet()) ? WordTable::individuals_table[object_.second->getValue()] : object_.first;
    }

    uint32_t getIdSubject() const { return subject_.second->getValue(); }

    uint32_t getIdProperty() const { return id_property_; }

    uint32_t getIdObject() const { return object_.second->getValue(); }

    std::pair<std::string, std::shared_ptr<Variable_t>> getSubject() const { return subject_; }

    std::pair<std::string, std::shared_ptr<Variable_t>> getObject() const { return object_; }

    bool getAdd() const { return add_; }

    bool isRequired() const { return required_; }

    TimeStamp_t getTimeStamp() const { return timestamp_; }

    uint32_t getId() const { return id_; }

    bool isValid() const { return id_property_ != 0; }

    std::string toString() const;

    std::string toShortString() const;

    friend std::ostream& operator<<(std::ostream& os, const Fact& fact);

    bool match(const Fact& other) const;

    void link(VariableTable_t& table_variables);


    void expandProperty(onto::ObjectPropertyClient* object_property_client);

private:
    bool add_;
    bool required_;
    std::pair<std::string, std::shared_ptr<Variable_t>> subject_;
    std::pair<std::string, std::shared_ptr<Variable_t>> object_;

    uint32_t id_;
    uint32_t id_property_;
    std::unordered_set<int32_t> id_extended_properties_;
    TimeStamp_t timestamp_;
    static uint32_t id_fact;
};

} // procedural

#endif //PROCEDURAL_FACT_H
