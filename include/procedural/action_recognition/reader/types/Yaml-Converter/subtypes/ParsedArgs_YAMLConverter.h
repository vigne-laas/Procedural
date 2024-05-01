#ifndef PROCEDURAL_PARSEDARGS_YAMLCONVERTER_H
#define PROCEDURAL_PARSEDARGS_YAMLCONVERTER_H

#include <yaml-cpp/yaml.h>
#include "procedural/action_recognition/reader/types/subtypes/ParsedArgs.h"

namespace YAML {
template<>
struct convert<procedural::ParsedArgs_t> {
    static Node encode(const procedural::ParsedArgs_t& rhs)
    {
        Node node;
        return node;
    }

    static bool decode(const Node& node, procedural::ParsedArgs_t& parsed_args)
    {
        if (!node.IsSequence()) {
            return false;
        }
        for (auto iter_node = 0; iter_node != node.size(); iter_node++)
        {
//            std::cout << "subnode args ";
            if (node[iter_node].IsMap())
            {
                for (const auto& map_elmt: node[iter_node])
                {
                 parsed_args.args[map_elmt.first.as<std::string>()] = map_elmt.second.as<std::string>();
                }
            }
        }
        return true;
    }
};

}
#endif //PROCEDURAL_PARSEDARGS_YAMLCONVERTER_H
