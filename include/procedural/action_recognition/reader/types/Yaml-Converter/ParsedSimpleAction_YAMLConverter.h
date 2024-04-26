#ifndef PROCEDURAL_PARSEDSIMPLEACTION_YAMLCONVERTER_H
#define PROCEDURAL_PARSEDSIMPLEACTION_YAMLCONVERTER_H

#include <yaml-cpp/yaml.h>
#include "procedural/action_recognition/reader/types/ParsedSimpleAction.h"
#include "procedural/action_recognition/reader/types/Yaml-Converter/subtypes/ParsedParameters_YAMLConverter.h"
#include "procedural/action_recognition/reader/types/Yaml-Converter/subtypes/ParsedDescription_YAMLConverter.h"
#include "procedural/action_recognition/reader/types/Yaml-Converter/subtypes/ParsedFacts_YAMLConverter.h"


namespace YAML {
template<>
struct convert<procedural::ParsedSimpleAction_t>
{
    static Node encode(const procedural::ParsedSimpleAction_t& rhs)
    {
        Node node;
        return node;
    }
    static bool decode(const Node& node, procedural::ParsedSimpleAction_t& new_simple_action)
    {
        if (!node.IsMap())
        {
            return false;
        }
        for (auto iter_node: node)
        {
            std::string key = iter_node.first.as<std::string>();
//            std::cout << "key : " << key << std::endl;
            if (key == "parameters")
                new_simple_action.parameters = iter_node.second.as<procedural::ParsedParameters_t>();
            if (key == "sequence")
                new_simple_action.facts = iter_node.second.as<procedural::ParsedFacts_t>();
            if (key == "description")
                new_simple_action.descriptions = iter_node.second.as<procedural::ParsedDescriptions_t>();
            //TODO if invalid key error

        }
        return true;
    }
};


}
#endif //PROCEDURAL_PARSEDSIMPLEACTION_YAMLCONVERTER_H
