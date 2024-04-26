#ifndef PROCEDURAL_FACT_H
#define PROCEDURAL_FACT_H

#include <string>
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>
#include "procedural/utils/TimeStamp.h"
#include "procedural/utils/WordTable.h"
#include "procedural/structures/Variable.h"

namespace procedural {
struct VariableTable_t;

class Fact {
public:


    Fact(bool add, const std::string& subject, const std::string& subject_type, const std::string& property,
         const std::string& object, const std::string& object_type, const TimeStamp_t& time);

    Fact(const Fact& other);

    Fact(bool add, const std::string& literal_subject,const std::string& subject_type, const std::string& property, const std::string& literal_object, const std::string& object_type);

    std::string getLiteralSubject() const { return subject_->literal_; }

    std::string getLiteralObject() const { return object_->literal_; }

    std::string getStrSubject() const
    {
        return (subject_->isSet()) ? individuals_table[subject_->getValue()] : subject_->literal_;
    }

    std::string getStrProperty() const { return properties_table[id_property_]; }

    std::string getStrObject() const
    {
        return (object_->isSet()) ? individuals_table[object_->getValue()] : object_->literal_;
    }

    uint32_t getIdSubject() const { return subject_->getValue(); }

    uint32_t getIdProperty() const { return id_property_; }

    uint32_t getIdObject() const { return object_->getValue(); }

    std::shared_ptr<Variable_t> getSubject() const { return subject_; }

    std::shared_ptr<Variable_t> getObject() const { return object_; }

    bool getAdd() const { return add_; }

    TimeStamp_t getTimeStamp() const { return timestamp_; }

    uint32_t getId() const { return id_; }

    bool isValid() const { return id_property_ != 0; }

    std::string toString() const;

    std::string toShortString() const;

    friend std::ostream& operator<<(std::ostream& os, const Fact& fact);

    bool match(const Fact& other) const;

    void link(VariableTable_t& table_variables);

    static WordTable properties_table;
    static WordTable individuals_table;

    void expandProperty(onto::ObjectPropertyClient* object_property_client);

private:
    bool add_;
    std::shared_ptr<Variable_t> subject_;
    std::shared_ptr<Variable_t> object_;

    uint32_t id_;
    uint32_t id_property_;
    std::unordered_set<int32_t> id_extended_properties_;
    TimeStamp_t timestamp_;
};

} // procedural

#endif //PROCEDURAL_FACT_H
