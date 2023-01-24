#ifndef PROCEDURAL_FACT_H
#define PROCEDURAL_FACT_H

#include <string>
#include <regex>
#include "procedural/core/WordTable.h"
//TODO Add stamp data et moyen de synchro les faits infered
//TODO add id
namespace procedural {

class Fact
{
public:
//    Fact(bool add, std::string subject, std::string property_, std::string object_);

    Fact(bool add, const std::string& subject, const std::string& property, const std::string& object);

    bool isValid() const;

    int32_t getProperty() const;

    uint32_t getSubject() const;

    uint32_t getObject() const;

    std::string getStringProperty() const;

    std::string getStringSubject() const;

    std::string getStringObject() const;

    void printProperties() const;

    void printSubjectsObjects() const;

    bool operator==(const Fact& F2);

    std::string toString() const;


    static WordTable table_properties_;
private:
    bool add_;
    uint32_t subject_;
    uint32_t property_;
    uint32_t object_;

    static WordTable table_subjects_objects_;
};

} // procedural

#endif //PROCEDURAL_FACT_H
