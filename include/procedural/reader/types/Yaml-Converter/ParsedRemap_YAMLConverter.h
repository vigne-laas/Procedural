#ifndef PROCEDURAL_PARSEDREMAP_YAMLCONVERTER_H
#define PROCEDURAL_PARSEDREMAP_YAMLCONVERTER_H

#include "procedural/reader/types/ParsedRemap.h"

namespace YAML {

template<>
struct convert<procedural::ParsedRemaps_t>
{
    static Node encode(const procedural::ParsedRemaps_t& rhs)
    {
        Node node;
        return node;
    }
    static bool decode(const Node& node, procedural::ParsedRemaps_t& parsed_remaps)
    {

        if (!node.IsSequence())
        {
            return false;
        }
        for (auto iter_node = 0; iter_node != node.size(); iter_node++)
        {
//            std::cout << "subnode remap ";
            if (node[iter_node].IsMap())
            {
                for (const auto& map_elmt: node[iter_node])
                {
//                    std::cout << "key : " << map_elmt.first.as<std::string>() << " value : "
//                              << map_elmt.second.as<std::string>() << std::endl;
                    auto origine = map_elmt.first.as<std::string>();
                    auto destination = map_elmt.second.as<std::string>();
                    parsed_remaps.addRemap(origine, destination);
                }
            }
        }

        return true;
    }
};

}
#endif //PROCEDURAL_PARSEDREMAP_YAMLCONVERTER_H
