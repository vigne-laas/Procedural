#include "procedural/reader/YamlReader.h"

#include <fstream>
#include <iostream>
#include <regex>
#include <filesystem>

namespace procedural {
//
//bool YamlReader::read(const std::string& path)
//{
//  std::ifstream config_file(path);
//  if(config_file.is_open())
//  {
//
//    std::vector<std::string> file;
//    std::string line;
//    while(std::getline(config_file,line))
//    {
//      removeComment(line);
//      file.push_back(line);
//    }
//
//    size_t current_line = 0;
//
//    elements_ = read(file, current_line);
//
//    config_file.close();
//    return true;
//  }
//  else
//    return false;
//}
//
//std::map<std::string, YamlElement> YamlReader::read(const std::vector<std::string>& lines, size_t& current_line)
//{
//  std::map<std::string, YamlElement> res;
//
//  std::regex element_regex(R"(^\s*([^\s]*)\s*:\s*([^\n]*)\s*$)"); // in case of bug, previous regex was (^\s*([^\s]*)\s*:\s*([^\n\s]*)\s*$)
//  std::regex list_regex(R"(^\s*-\s*(.*)\s*$)");
//  std::smatch match;
//
//  std::string config_name;
//
//  size_t nb_spaces_base = countSpaces(lines[current_line]);
//
//  do
//  {
//    if(countSpaces(lines[current_line]) < nb_spaces_base)
//      return res;
//
//    if(std::regex_match(lines[current_line], match, element_regex))
//    {
//      if(match[2].str() == "")
//      {
//        config_name = match[1].str();
//        res[config_name] = YamlElement();
//        size_t next_nb_spaces = countSpaces(lines[current_line + 1]);
//        if(next_nb_spaces > nb_spaces_base)
//        {
//          if(std::regex_match(lines[current_line + 1], match, list_regex))
//          {
//            res[config_name].data = std::vector<std::string>();
//
//            do
//            {
//              current_line++;
//              res[config_name].data->push_back(match[1].str());
//            }
//            while((current_line + 1 < lines.size()) && (std::regex_match(lines[current_line + 1], match, list_regex)));
//          }
//          else
//          {
//            current_line++;
//            res[config_name].subelem = read(lines, current_line);
//            current_line--;
//          }
//        }
//      }
//      else
//      {
//        config_name = match[1].str();
//        res[config_name] = YamlElement();
//        res[config_name].data = std::vector<std::string>();
//        res[config_name].data->push_back(match[2].str());
//      }
//    }
//
//    current_line++;
//  }
//  while(current_line < lines.size());
//
//  return res;
//}
//
//void YamlReader::display()
//{
//  display(elements_);
//}
//
//void YamlReader::display(const std::map<std::string, YamlElement>& config, size_t nb)
//{
//  if((nb == 0) && (config.find("modules") != config.end()))
//    displayElement(*config.find("modules"), nb);
//
//  for(auto& c : config)
//  {
//    if((nb == 0) && (c.first == "modules"))
//      continue;
//
//    displayElement(c, nb);
//  }
//}
//
//void YamlReader::displayElement(const std::pair<std::string, YamlElement>& it, size_t nb)
//{
//  displayTab(nb);
//  std::cout << "\e[1m" << it.first << "\e[0m : ";
//  if(it.second.data)
//  {
//    if(it.second.data.value().size() > 1)
//    {
//      std::cout << std::endl;
//      for(auto& d : it.second.data.value())
//      {
//        displayTab(nb+1);
//        std::cout << "- " << d << std::endl;
//      }
//    }
//    else
//      std::cout << it.second.data.value().front() << std::endl;
//  }
//  else if(it.second.subelem)
//  {
//    std::cout << std::endl;
//    display(it.second.subelem.value(), nb+1);
//  }
//}
//
//void YamlReader::displayTab(size_t nb)
//{
//  for(size_t i = 0; i < nb; i++)
//    std::cout << "\t";
//}
//
//void YamlReader::removeComment(std::string& line)
//{
//  size_t pose = line.find('#');
//  if(pose != std::string::npos)
//    line = line.substr(0, pose);
//}
//
//size_t YamlReader::countSpaces(const std::string& line)
//{
//  size_t nb = 0;
//  while((nb < line.size()) && ((line[nb] == ' ') || (line[nb] == '\t')) )
//    nb++;
//
//  return nb;
//}
//
//Action::Action(const YAML::Node& yamlAction, const std::string name) : patternDescription(
//        R"(\s*\?([^\s]*)\s+([^\s]*)\s+\??([^\s]*)\s*)"), patternFacts(
//        R"(\s*(NOT)?\s*\?([^\s]*)\s+([^\s]*)\s+\?([^\s]*)\s*(REQUIRED)?)"), name_(name)
//{
////    name_ = name;
////    patternFacts = "\\s*(NOT)?\\s*\\?([^\\s]*)\\s+([^\\s]*)\\s+\\?([^\\s]*)\\s*(REQUIRED)?";
//    std::cout << "Create Action : " << name_ << std::endl;
//    if (yamlAction["ordered_facts"])
//    {
//        ordered_ = true;
//        readFacts(yamlAction["ordered_facts"]);
//    } else
//    {
//        if (yamlAction["unordered_facts"])
//            readFacts(yamlAction["unordered_facts"]);
//        else
//            std::cerr << " Action without facts" << std::endl;
//    }
//    if (yamlAction["description"])
//        readDescriptions(yamlAction["description"]);
//    else
//        std::cerr << " Action without description" << std::endl;
//}
//
//const std::vector<std::string>& Action::getFacts()
//{
//    return facts_;
//}
//
//const std::vector<std::string>& Action::getDescriptions()
//{
//    return descriptions_;
//}
//
//const std::vector<std::string>& Action::getVariables()
//{
//    return variables_;
//}
//
//void Action::readDescriptions(const YAML::Node& yamlDescriptions)
//{
//    std::cout << "Description of  " << name_ << " : " << std::endl;
//    if (yamlDescriptions.IsSequence())
//    {
//        for (auto i = 0; i < yamlDescriptions.size(); i++)
//        {
//            std::cout << yamlDescriptions[i].as<std::string>() << std::endl;
//        }
//    }
//
//}
//
//void Action::readFacts(const YAML::Node& yamlFacts)
//{
//    if (yamlFacts.IsSequence())
//    {
//        for (const auto & yamlFact : yamlFacts)
//        {
//            if (yamlFact.IsScalar())
//                std::cout << yamlFact.as<std::string>() << std::endl;
//
//            else if (yamlFact["or_facts"])
//            {
//                std::cout << "or facts : " << std::endl;
//                for (auto j = 0; j < yamlFact["or_facts"].size(); j++)
//                {
//                    std::cout << "      " << yamlFact["or_facts"][j].as<std::string>() << std::endl;
//                }
//
//            } else
//                std::cerr << "invalid synthax" << std::endl;
////                std::cout << "other type : sequence " << yamlFacts[i].IsSequence() << " map " << yamlFacts[i].IsMap() << " name : " << yamlFacts[i] << std::endl;
//        }
//    }
//
//}
//
//void Action::parseFact(const std::string& fact)
//{
////    std::smatch matches;
////    if(std::regex_search(fact,matches,patternFacts))
////    {
////
////    }
//}

bool YamlReader::read(const std::string& path)
{
    if (std::filesystem::exists(path)) //TODO demander pour l'import de experimental
    {
        YAML::Node yamlFile = YAML::LoadFile(path);
        for (YAML::const_iterator it = yamlFile.begin(); it != yamlFile.end(); ++it)
        {
            std::cout << "Action find " << it->first.as<std::string>() << std::endl;
    //        Action(yamlFile[it->first.as<std::string>()], it->first.as<std::string>());
        }
    }
    return true;
}
} // namespace procedural