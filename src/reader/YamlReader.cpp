#include <filesystem>
#include <iostream>
#include "procedural/reader/YamlReader.h"
#include "procedural/reader/types/Yaml-Converter/ParsedComposedAction_YAMLConverter.h"
#include "procedural/reader/types/Yaml-Converter/ParsedSimpleAction_YAMLConverter.h"
namespace procedural {
YamlReader::YamlReader() : yaml_file_()
{

}

bool YamlReader::read(const std::string& path)
{
    if (std::filesystem::exists(path))
    {
        yaml_file_ = YAML::LoadFile(path);
        parse();
    } else
        return false;
    return true;
}
// ------------------------------------------------------------------ private part ----------------------------------------------------------- //

bool YamlReader::parse()
{
    for (auto it = yaml_file_.begin(); it != yaml_file_.end(); ++it)
    {
        if (isSimpleAction(it->second))
        {
            auto action = it->second.as<ParsedSimpleAction_t>();
            action.name_ = it->first.as<std::string>();
            simple_actions_.push_back(action);
        }
        if (isComposedAction(it->second))
        {
            auto action = it->second.as<ParsedComposedAction_t>();
            action.name_ = it->first.as<std::string>();
            composed_actions_.push_back(action);
        }
    }
    return (composed_actions_.empty() == false) || (simple_actions_.empty() == false);
}


} // procedural