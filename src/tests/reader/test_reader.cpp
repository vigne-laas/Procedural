#include "test_reader.h"

#include "procedural/reader/YamlReader.h"
#include <iostream>
#include "yaml.h"
#include <yaml-cpp/yaml.h>
int main()
{
//    YAML::Node node = YAML::LoadFile("/home/avigne/Projects/Procedural/catkin_ws/src/procedural/src/reader/exemple.yaml");
//    std::cout<< "size : " << node.size()<<  " Map ? " << node.IsMap() << " Sequence ? " << node.IsSequence() << std::endl;
//    for(YAML::const_iterator it=node.begin();it!=node.end();++it) {
//        std::cout << "Playing at " << it->first.as<std::string>()<< std::endl;
//    }
//    for (std::size_t i=0;i<node.size();i++) {
//        std::cout << node[i].Tag() << "\n";
//    }
    procedural::YamlReader reader = procedural::YamlReader();
    reader.read("/home/avigne/Projects/Procedural/catkin_ws/src/procedural/src/reader/exemple.yaml");

//    procedural::YamlReader reader;
//    reader.read("/home/avigne/Projects/Procedural/catkin_ws/src/procedural/src/reader/exemple.yaml");
//    reader.display();
//    for (auto key: reader.getKeys())
//    {
//        std::cout << "key : " << key << " => "<< std::endl;
//        for (auto val: reader[key].getElementsKeys())
//        {
//            std::cout << "      value : " << val<<std::endl;
//            for(auto subsubelmt : reader[key][val].value())
//                std::cout << "              subsubelmt : " << subsubelmt<<std::endl;
//            std::cout << "key of sub element : " << val[]
//        }
//        std::cout << "/n/n" << std::endl;
//    }

    return 0;
}