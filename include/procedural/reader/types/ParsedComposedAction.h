#ifndef PROCEDURAL_PARSEDCOMPOSEDACTION_H
#define PROCEDURAL_PARSEDCOMPOSEDACTION_H
#include "procedural/reader/types/ParsedPattern.h"
#include "procedural/reader/types/ParsedDescription.h"
#include "procedural/reader/types/ParsedParameters.h"
#include "procedural/reader/types/ParsedRemap.h"


namespace procedural {
struct ParsedComposedAction_t
{
    ParsedComposedAction_t() = default;

    std::string name_;
    procedural::ParsedParameters_t parameters;
    procedural::ParsedPattern_t pattern;
    procedural::ParsedDescriptions_t descriptions;
    procedural::ParsedRemaps_t remaps;

    void linkRemapPattern()
    {
        for (auto& remap_element: remaps.remaps)
        {
            auto literal = remap_element.first;
            auto find = [literal](const SubNetwork_t& subnet) { return subnet.literal == literal; };
            auto result = std::find_if(pattern.subnetworks.begin(), pattern.subnetworks.end(), find);
            if (result != pattern.subnetworks.end())
                result->remap = remap_element.second.remap;
            else
                std::cout << "Warning : not network find for this remap literal : " << literal << std::endl;
        }


    }

    void addPattern(const ParsedPattern_t& new_pattern)
    {
        pattern = new_pattern;
        if (remaps.empty() == false)
            linkRemapPattern();
    }
    void addRemap(const ParsedRemaps_t& new_remap)
    {
        remaps = new_remap;
        if (pattern.empty() == false)
            linkRemapPattern();
    }

    friend std::ostream& operator<<(std::ostream& os, const ParsedComposedAction_t& lhs)
    {
        os << "Composed Action : " << lhs.name_ << "\n";
        os << lhs.pattern;
        os << "Description : \n" << lhs.descriptions;
        os << ((lhs.parameters.empty()) ? "" : "Parameters : \n") << lhs.parameters;

        return os;
    }
};
}



#endif //PROCEDURAL_PARSEDCOMPOSEDACTION_H
