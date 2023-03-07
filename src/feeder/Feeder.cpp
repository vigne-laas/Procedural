//
// Created by avigne on 18/01/23.
//

#include "procedural/feeder/Feeder.h"
#include <iostream>

namespace procedural {

Feeder::Feeder() : pattern("\\[([^\\]]+)\\]([^|]+)\\|([^|]+)\\|(.+)"){

}


procedural::Fact* Feeder::parse(const std::string& fact)
{
    std::smatch matches;
    if (std::regex_search(fact, matches, pattern))
    {
        std::cout << "match ok " << std::endl;
//        for (auto i = 0; i < matches.size(); ++i)
//        {
//            std::cout << i << ": '" << matches[i].str() << "'" << std::endl;
//        }
//        if (matches.size() == 5)
//        {
//
//        }
        if (matches[1].str() == "DEL")
            return new Fact(false, matches[2], matches[3], matches[4],0); // adapt to take into account id 
        else
            return new Fact(true, matches[2], matches[3], matches[4],0);

    }
    return nullptr;
}


} // procedural