#ifndef PROCEDURAL_PARSEDFACTS_H
#define PROCEDURAL_PARSEDFACTS_H

#include <regex>
#include <ontologenius/OntologyManipulator.h>
#include <unordered_set>

namespace procedural {

struct ParsedFact_t {
    ParsedFact_t()
            : subject(), property(), object(), insertion(false), required(false), level(0),
              subject_is_variable(false), object_is_variable(false),
              regex_facts_(R"(\s*(NOT)?\s*\?([^\s]*)\s+([^\s]*)\s+\?([^\s]*)\s*(REQUIRED)?)") {}

    ParsedFact_t(const std::string& str_value, uint32_t level)
            : subject(), property(), object(), insertion(false), required(false), level(level),
              subject_is_variable(false), object_is_variable(false),
              regex_facts_(R"(\s*(NOT)?\s*\?([^\s]*)\s+([^\s]*)\s+\?([^\s]*)\s*(REQUIRED)?)")
    {
        parse(str_value);
    }

    void parse(std::string str_value)
    {
        std::smatch results;
        std::regex_search(str_value, results, regex_facts_);
        insertion = (results[1] != "NOT");
        subject = results[2];
        property = results[3];
        object = results[4];
        required = results[5].str() == "REQUIRED";
        // La regex exige un '?' devant le sujet et l'objet, donc ce sont toujours des variables
        subject_is_variable = true;
        object_is_variable = true;
    }

    void complete(onto::OntologyManipulator& ontology_manip)
    {
        auto str_type = ontology_manip.classes.getUp(subject, 1);
        subject_type = str_type.front();
        for (const auto& type: str_type)
            for (const auto& type_extended: ontology_manip.classes.getDown(type))
                type_subject_extended.insert(type_extended);
        str_type = ontology_manip.classes.getUp(object, 1);
        object_type = str_type.front();
        for (const auto& type: str_type)
            for (const auto& type_extended: ontology_manip.classes.getDown(type))
                type_subject_extended.insert(type_extended);

    }

    friend std::ostream& operator<<(std::ostream& os, const ParsedFact_t& lhs)
    {
        os << lhs.toString();
        return os;
    }

    std::regex regex_facts_;
    std::string subject;
    std::string subject_type;
    std::unordered_set<std::string> type_subject_extended;
    std::string property;
    std::string object;
    std::string object_type;
    bool insertion;
    bool required;
    int level;
    bool subject_is_variable;
    bool object_is_variable;

    std::string toString() const
    {
        std::string str = ((insertion) ? "[ADD] " : "[DEL] ");
        str += subject;
        if (!subject_type.empty()){
            if (!type_subject_extended.empty())
            {
                str += " (";
                for (const auto& type_subject: type_subject_extended)
                {
                    str += type_subject + ", ";
                }
                str.pop_back();
                str.pop_back();
                str += ")";
            }
            else
                str += " (" + subject_type + ")";
        }

        str += " " + property + " ";
        str += object;
        if (!object_type.empty()){
            if (!type_subject_extended.empty())
            {
                str += " (";
                for (const auto& type_object: type_subject_extended)
                {
                    str += type_object + ", ";
                }
                str.pop_back();
                str.pop_back();
                str += ")";
            }
            else
                str += " (" + object_type + ")";
        }
        str += ((required) ? " Required " : " ");
        str += "level : " + std::to_string(level);
        return str;
    }
};

struct ParsedFacts_t {
    ParsedFacts_t() {}

    friend std::ostream& operator<<(std::ostream& os, const ParsedFacts_t& lhs)
    {
        for (const auto& fact: lhs.facts_)
            os << fact << "\n";
        return os;
    }

    std::vector<ParsedFact_t> facts_;
};

} // namespace procedural

#endif //PROCEDURAL_PARSEDFACTS_H
