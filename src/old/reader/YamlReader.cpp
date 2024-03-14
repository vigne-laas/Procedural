#include <filesystem>
#include <iostream>
#include "procedural/old/reader/YamlReader.h"
#include "procedural/old/reader/types/Yaml-Converter/ParsedComposedAction_YAMLConverter.h"
#include "procedural/old/reader/types/Yaml-Converter/ParsedSimpleAction_YAMLConverter.h"

namespace procedural {

bool YamlReader::read(const std::string& path)
{
    if (std::filesystem::exists(path))
    {
        yaml_file_ = YAML::LoadFile(path);
        return parse();
    }
    else
        return false;
}
// ------------------------------------------------------------------ private part ----------------------------------------------------------- //

bool YamlReader::parse()
{
    for (auto it = yaml_file_.begin(); it != yaml_file_.end(); ++it)
    {
        if (isSimpleAction(it->second))
        {
            auto action = it->second.as<ParsedSimpleAction_t>();
            action.setType(it->first.as<std::string>());
            simple_actions_.push_back(action);
        }
        if (isComposedAction(it->second))
        {
            auto action = it->second.as<ParsedComposedAction_t>();
            action.setType(it->first.as<std::string>());
            composed_actions_.push_back(action);
        }
    }
    return (composed_actions_.empty() == false) || (simple_actions_.empty() == false);
}

bool YamlReader::isSimpleAction(const YAML::Node& node)
{
    return node["sequence"] && node["description"];
}

bool YamlReader::isComposedAction(const YAML::Node& node)
{
    return node["composed_sequence"] && node["description"];
}

} // namespace procedural