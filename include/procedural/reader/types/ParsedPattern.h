#ifndef PROCEDURAL_PARSEDPATTERN_H
#define PROCEDURAL_PARSEDPATTERN_H

#include "procedural/reader/types/ParsedFacts.h"

namespace procedural {
struct SubNetwork_t
{
    SubNetwork_t() : level(0), regex_type(R"(\s*([^\s]*)\s*(REQUIRED)?)")
    {};
    SubNetwork_t(const std::string& new_literal, const std::string& new_type, uint32_t new_level) : literal(
            new_literal),
                                                                                                    level(new_level),
                                                                                                    regex_type(R"(\s*([^\s]*)\s*(REQUIRED)?)")
    {
        std::smatch results;
        std::regex_search(new_type, results, regex_type);
        type = results[1];
        (results[2] == "") ? required = false : required = true;
    };

    friend std::ostream& operator<<(std::ostream& os, const SubNetwork_t& lhs)
    {

        os << lhs.literal << " <=> " << lhs.type << " / level :  " << lhs.level;
        os << (lhs.required ? " REQUIRED \n" : "\n");
        for (auto& map_elmt: lhs.remap)
            os << map_elmt.first << " => " << map_elmt.second << "\n";
        return os;
    }
    std::regex regex_type;
    std::string literal;
    std::string type;
    std::map<std::string, std::string> remap;
    int level;
    bool required;
};

struct ParsedPattern_t
{
    ParsedPattern_t() = default;

    friend std::ostream& operator<<(std::ostream& os, const ParsedPattern_t& lhs)
    {
        if (lhs.empty())
            return os;
        os << "subnet parsed : \n";
        for (auto& subnet: lhs.subnetworks)
            os << subnet << "\n";
        os << "facts parsed : \n";
        for (auto& fact: lhs.facts)
            os << fact << "\n";
        return os;
    }

    bool empty() const
    { return subnetworks.empty(); }

    std::vector<SubNetwork_t> subnetworks;
    std::vector<ParsedFact_t> facts;
};
}

#endif //PROCEDURAL_PARSEDPATTERN_H
