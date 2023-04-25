#ifndef PROCEDURAL_PARSEDSIMPLEACTION_H
#define PROCEDURAL_PARSEDSIMPLEACTION_H
#include "procedural/reader/types/ParsedParameters.h"
#include "procedural/reader/types/ParsedDescription.h"
#include "procedural/reader/types/ParsedFacts.h"
namespace procedural {
struct ParsedSimpleAction_t
{
    ParsedSimpleAction_t()
    {};

    std::string name_;
    ParsedParameters_t parameters;
    ParsedFacts_t facts;
    ParsedDescriptions_t descriptions;


    friend std::ostream& operator<<(std::ostream& os, const ParsedSimpleAction_t& lhs)
    {
        os << "Simple Action : " << lhs.name_ << "\n";
        os << lhs.facts;
        os << "Description : \n" << lhs.descriptions;
        os << ((lhs.parameters.empty()) ? "" : "Parameters : \n") << lhs.parameters;

        return os;
    }
};
}



#endif //PROCEDURAL_PARSEDSIMPLEACTION_H
