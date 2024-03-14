#ifndef PROCEDURAL_PARSEDCOMPOSEDACTION_YAMLCONVERTER_H
#define PROCEDURAL_PARSEDCOMPOSEDACTION_YAMLCONVERTER_H

#include <yaml-cpp/yaml.h>
#include "procedural/old/reader/types/ParsedComposedAction.h"
#include "procedural/old/reader/types/Yaml-Converter/ParsedDescription_YAMLConverter.h"
#include "procedural/old/reader/types/Yaml-Converter/ParsedParameters_YAMLConverter.h"
#include "procedural/old/reader/types/Yaml-Converter/ParsedPattern_YAMLConverter.h"
#include "procedural/old/reader/types/Yaml-Converter/ParsedRemap_YAMLConverter.h"

namespace YAML {
template<>
struct convert<procedural::ParsedComposedAction_t>
{
    static Node encode(const procedural::ParsedComposedAction_t& rhs)
    {
        Node node;
        return node;
    }
    static bool decode(const Node& node, procedural::ParsedComposedAction_t& new_composed_action)
    {
        if (!node.IsMap())
        {
            return false;
        }
        for (auto iter_node: node)
        {
            std::string key = iter_node.first.as<std::string>();
            if (key == "parameters")
                new_composed_action.parameters = iter_node.second.as<procedural::ParsedParameters_t>();
            if (key == "composed_sequence")
                new_composed_action.addPattern(iter_node.second.as<procedural::ParsedPattern_t>());
            if (key == "description")
                new_composed_action.descriptions = iter_node.second.as<procedural::ParsedDescriptions_t>();
            if (key == "remap")
                new_composed_action.addRemap(iter_node.second.as<procedural::ParsedRemaps_t>());
        }

        return true;
    }
};

}
#endif //PROCEDURAL_PARSEDCOMPOSEDACTION_YAMLCONVERTER_H
