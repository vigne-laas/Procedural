#ifndef PROCEDURAL_PARSEDCOMPOSEDACTION_YAMLCONVERTER_H
#define PROCEDURAL_PARSEDCOMPOSEDACTION_YAMLCONVERTER_H

#include <yaml-cpp/yaml.h>
#include "procedural/action_recognition/reader/types/Yaml-Converter/subtypes/ParsedPattern_YAMLConverter.h"
#include "procedural/action_recognition/reader/types/Yaml-Converter/subtypes/ParsedRemap_YAMLConverter.h"
#include "procedural/action_recognition/reader/types/Yaml-Converter/subtypes/ParsedParameters_YAMLConverter.h"
#include "procedural/action_recognition/reader/types/Yaml-Converter/subtypes/ParsedDescription_YAMLConverter.h"

namespace YAML {
template<>
struct convert<action_recognition::ParsedComposedAction_t>
{
    static Node encode(const action_recognition::ParsedComposedAction_t& rhs)
    {
        Node node;
        return node;
    }
    static bool decode(const Node& node, action_recognition::ParsedComposedAction_t& new_composed_action)
    {
        if (!node.IsMap())
        {
            return false;
        }
        for (auto iter_node: node)
        {
            std::string key = iter_node.first.as<std::string>();
            if (key == "parameters")
                new_composed_action.parameters = iter_node.second.as<action_recognition::ParsedParameters_t>();
            if (key == "composed_sequence")
                new_composed_action.addPattern(iter_node.second.as<action_recognition::ParsedPattern_t>());
            if (key == "description")
                new_composed_action.descriptions = iter_node.second.as<action_recognition::ParsedDescriptions_t>();
            if (key == "remap")
                new_composed_action.addRemap(iter_node.second.as<action_recognition::ParsedRemaps_t>());
        }

        return true;
    }
};

}
#endif //PROCEDURAL_PARSEDCOMPOSEDACTION_YAMLCONVERTER_H
