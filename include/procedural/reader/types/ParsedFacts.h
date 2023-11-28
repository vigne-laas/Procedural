#ifndef PROCEDURAL_PARSEDFACTS_H
#define PROCEDURAL_PARSEDFACTS_H

#include <regex>

namespace procedural {

struct ParsedFact_t
{
    ParsedFact_t()
            : subject(), property(), object(), insertion(false), required(false), level(0),
              regex_facts_(R"(\s*(NOT)?\s*\?([^\s]*)\s+([^\s]*)\s+\?([^\s]*)\s*(REQUIRED)?)")
    {}
    ParsedFact_t(const std::string& str_value, uint32_t level)
            : subject(), property(), object(), insertion(false), required(false), level(level),
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
    }

    friend std::ostream& operator<<(std::ostream& os, const ParsedFact_t& lhs)
    {
        os << ((lhs.insertion) ? "[ADD] " : "[DEL] ");
        os << lhs.subject << " " << lhs.property << " " << lhs.object << " " << ((lhs.required) ? " Required " : "");
        os << "level : " << lhs.level ;
        return os;
    }

    std::regex regex_facts_;
    std::string subject;
    std::string property;
    std::string object;
    bool insertion;
    bool required;
    int level;
};

struct ParsedFacts_t
{
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
