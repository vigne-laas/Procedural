#ifndef PROCEDURAL_PARSEDPARAMETERS_YAMLCONVERTER_H
#define PROCEDURAL_PARSEDPARAMETERS_YAMLCONVERTER_H
#include <yaml-cpp/yaml.h>
#include "procedural/action_recognition/reader/types/subtypes/ParsedParameters.h"
namespace YAML {

template<>
struct convert<action_recognition::ParsedParameters_t>
{
    static Node encode(const action_recognition::ParsedParameters_t& rhs)
    {
        Node node;
        return node;
    }
    static bool decode(const Node& node, action_recognition::ParsedParameters_t& parameters)
    {
        if (!node.IsMap())
        {
            return false;
        }

        for (auto iter_node: node)
        {
            auto key = iter_node.first.as<std::string>();
//            std::cout << "key parameters: " << key << std::endl;
            if (key == "ttl")
                parameters.ttl = iter_node.second.as<double>();

        }
        return true;
    }
};

}

#endif //PROCEDURAL_PARSEDPARAMETERS_YAMLCONVERTER_H
