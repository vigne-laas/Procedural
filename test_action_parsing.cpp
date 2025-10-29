#include <iostream>
#include "procedural/memory/ProceduralFullReader.h"

int main()
{
    std::cout << "Testing action parsing with domain3.dom..." << std::endl;

    procedural::ProceduralFullReader reader;
    std::string domain_path = "/home/avigne/Projets/ArchiThese/catkin_ws/src/restaurant_resources/domaines/v2/domain3.dom";

    try {
        if (reader.read(domain_path))
        {
            std::cout << "✓ Successfully parsed domain3.dom" << std::endl;

            auto actions = reader.getActions();
            std::cout << "Parsed " << actions.actions.size() << " actions." << std::endl;

            for (const auto& action : actions.actions)
            {
                std::cout << "- Action: " << action.name << " (";
                for (size_t i = 0; i < action.arguments.size(); ++i) {
                    if (i > 0) std::cout << ", ";
                    std::cout << action.arguments[i].type << " " << action.arguments[i].literal;
                }
                std::cout << ")" << std::endl;

                std::cout << "  Preconditions: " << action.preconditions.size() << std::endl;
                std::cout << "  Effects: " << action.effects.size() << std::endl;
                std::cout << "  Execution actions: " << action.executions_bloc.size() << std::endl;
                std::cout << std::endl;
            }
        }
        else
        {
            std::cout << "✗ Failed to parse domain3.dom" << std::endl;
            return 1;
        }
    }
    catch (const std::exception& e)
    {
        std::cout << "✗ Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}