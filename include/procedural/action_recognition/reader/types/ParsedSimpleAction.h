#ifndef PROCEDURAL_PARSEDSIMPLEACTION_H
#define PROCEDURAL_PARSEDSIMPLEACTION_H

#include <regex>
#include <iostream>
#include "procedural/action_recognition/reader/types/subtypes/ParsedParameters.h"
#include "procedural/action_recognition/reader/types/subtypes/ParsedFacts.h"
#include "procedural/action_recognition/reader/types/subtypes/ParsedDescription.h"
#include "procedural/action_recognition/reader/types/subtypes/ParsedArgs.h"

namespace procedural {

struct ParsedSimpleAction_t
{
    ParsedSimpleAction_t() : regex_type(R"(\s*([^_\s]*)_?([^\s]*)?\s*)") {}


    ParsedArgs_t args;
    std::regex regex_type;
    std::string type;
    std::string subtype;
    ParsedParameters_t parameters;
    ParsedFacts_t facts;
    ParsedDescriptions_t descriptions;

    std::string getName() const
    {
        return subtype.empty() ? type : type + "_" + subtype;
    }

    friend std::ostream& operator<<(std::ostream& os, const ParsedSimpleAction_t& lhs)
    {
        os << "Simple Action : " << lhs.type;
        os << (lhs.subtype.empty() ? "\n" : "_" + lhs.subtype + "\n");
        os << "Args : \n" << lhs.args;
        os << lhs.facts;
        os << "Description : \n" << lhs.descriptions;
        os << ((lhs.parameters.empty()) ? "" : "Parameters : \n") << lhs.parameters;

        return os;
    }

    void setType(const std::string& str_type)
    {
        std::smatch results;
        std::regex_search(str_type, results, regex_type);
        type = results[1];
        subtype = results[2];
    }
};

}

#endif //PROCEDURAL_PARSEDSIMPLEACTION_H
