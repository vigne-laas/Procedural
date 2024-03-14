#ifndef PROCEDURAL_PARSEDPARAMETERS_YAMLCONVERTER_H
#define PROCEDURAL_PARSEDPARAMETERS_YAMLCONVERTER_H

#include "procedural/old/reader/types/ParsedParameters.h"

namespace YAML {

template<>
struct convert<procedural::ParsedParameters_t>
{
    static Node encode(const procedural::ParsedParameters_t& rhs)
    {
        Node node;
        return node;
    }
    static bool decode(const Node& node, procedural::ParsedParameters_t& parameters)
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
