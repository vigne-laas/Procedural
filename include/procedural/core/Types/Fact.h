#ifndef PROCEDURAL_FACT_H
#define PROCEDURAL_FACT_H

#include <string>
#include <regex>

#include "procedural/core/Types/WordTable.h"
#include "procedural/utils/TimeStamp.h"


//TODO Add stamp data et moyen de synchro les faits infered
//TODO add id
namespace procedural {

class Fact
{
public:
    Fact(bool add, const std::string& subject, const std::string& property, const std::string& object, uint32_t id, const TimeStamp_t & time);

    bool isValid() const { return property_ != 0; }

    int32_t getProperty() const { return (int32_t)property_ * (add_ ? 1 : -1); }
    uint32_t getSubject() const { return subject_; }
    uint32_t getObject() const { return object_; }
    uint32_t getId() const { return id_; }
    TimeStamp_t getTimeStamp() const { return timestamp_; }

    std::string getStringProperty() const { return properties_table[property_]; }
    std::string getStringSubject() const { return individuals_table[subject_]; }
    std::string getStringObject() const { return individuals_table[object_]; }

    bool operator==(const Fact& other) const;

    std::string toString() const;

    static WordTable properties_table;
    static WordTable individuals_table;

private:
    bool add_;
    uint32_t subject_;
    uint32_t property_;
    uint32_t object_;
    uint32_t id_;
    TimeStamp_t timestamp_;


};

} // procedural

#endif //PROCEDURAL_FACT_H
