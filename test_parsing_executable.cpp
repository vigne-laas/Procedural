#include <iostream>
#include "procedural/memory/ProceduralFullReader.h"

int main()
{
    std::cout << "Testing priority parsing with domain3.dom..." << std::endl;

    procedural::ProceduralFullReader reader;
    std::string domain_path = "/home/avigne/Projets/ArchiThese/catkin_ws/src/restaurant_resources/domaines/v2/domain3.dom";

    try {
        if (reader.read(domain_path))
        {
            std::cout << "✓ Successfully parsed domain3.dom" << std::endl;

            auto priorities = reader.getPriorities();
            std::cout << "Found " << priorities.size() << " priorities:" << std::endl;

            for (const auto& priority : priorities)
            {
                std::cout << "\n--- Priority: " << priority->name << " ---" << std::endl;
                std::cout << "Level: " << priority->level << std::endl;

                std::cout << "Preconditions (" << priority->preconditions.size() << "):" << std::endl;
                for (const auto& precond : priority->preconditions)
                {
                    std::cout << "  " << precond << std::endl;
                }

                std::cout << "Legacy objectives (" << priority->objectives.size() << "):" << std::endl;
                for (const auto& obj : priority->objectives)
                {
                    std::cout << "  " << obj << std::endl;
                }

                std::cout << "State objectives (" << priority->state_objectives.size() << "):" << std::endl;
                for (const auto& state_obj : priority->state_objectives)
                {
                    std::cout << "  " << state_obj << std::endl;
                }

                if (!priority->task_name.empty())
                {
                    std::cout << "Task: " << priority->task_name << std::endl;
                    std::cout << "Task parameters (" << priority->task_parameters.size() << "):" << std::endl;
                    for (const auto& param : priority->task_parameters)
                    {
                        std::cout << "  " << param << std::endl;
                    }
                }
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