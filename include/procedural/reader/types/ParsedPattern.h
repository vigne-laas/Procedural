#ifndef PROCEDURAL_PARSEDPATTERN_H
#define PROCEDURAL_PARSEDPATTERN_H

#include "procedural/reader/types/ParsedFacts.h"

namespace procedural {
struct SubStateMachine_t
{
    SubStateMachine_t() : level(0), regex_type(R"(\s*([^\s]*)\s*(REQUIRED)?)")
    {};
    SubStateMachine_t(const std::string& new_literal, const std::string& new_type, int new_level) : literal(
            new_literal),
                                                                                                    level(new_level),
                                                                                                    regex_type(
                                                                                                       R"(\s*([^_\s]*)_?([^\s]*)?\s*(REQUIRED)?)")
    {
        std::smatch results;
        std::regex_search(new_type, results, regex_type);
//        for (auto res: results)
//            std::cout << "res :" << res << std::endl;
        type = results[1];
        sub_type = results[2];
        (results[3] == "") ? required = false : required = true;
    };

    std::string getName() const
    {
        std::string res = type;
        return sub_type.empty() ? res : res + "_" + sub_type;
    }

    friend std::ostream& operator<<(std::ostream& os, const SubStateMachine_t& lhs)
    {

        os << lhs.literal << " <=> " << lhs.type;
        os << (lhs.sub_type.empty() ? "" : ("_" + lhs.sub_type)) << " / level :  " << lhs.level;
        os << (lhs.required ? " REQUIRED \n" : "\n");
        for (auto& map_elmt: lhs.remap)
            os << map_elmt.first << " => " << map_elmt.second << "\n";
        return os;
    }
    std::regex regex_type;
    std::string literal;
    std::string type;
    std::string sub_type;
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
        for (auto& subnet: lhs.sub_state_machines)
            os << subnet << "\n";
        os << "facts parsed : \n";
        for (auto& fact: lhs.facts)
            os << fact << "\n";
        return os;
    }

    bool empty() const
    { return sub_state_machines.empty(); }

    std::vector<SubStateMachine_t> sub_state_machines;
    std::vector<ParsedFact_t> facts;
    int max_level;
};
}

#endif //PROCEDURAL_PARSEDPATTERN_H
