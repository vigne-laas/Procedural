#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include "procedural/task_recognition/Reader/DomainReader.h"
#include "procedural/utils/Logger.h"

namespace procedural {

TEST(ForClauseParsingTest, testForClauseParsing) {
    // Create domain reader
    DomainReader reader;

    // Parse the test domain
    std::string package_path = ros::package::getPath("procedural");
    std::string domain_path = package_path + "/test/test_attribution_domain.dom";

    std::cout << "Loading domain: " << domain_path << std::endl;

    bool read_result = reader.read(domain_path);
    ASSERT_TRUE(read_result) << "Failed to parse domain!";

    std::cout << "\n✓ Domain parsed successfully" << std::endl;

    auto actions = reader.getActions();
    std::cout << "Number of actions: " << actions.size() << std::endl;
    ASSERT_GT(actions.size(), 0) << "No actions parsed";

    // Test each action's commitments
    for (const auto& action : actions) {
        std::cout << "\n--- Action: " << action.name << " ---" << std::endl;

        if (!action.commitments.has_commitments) {
            std::cout << "  No commitments for this action" << std::endl;
            continue;
        }

        // Test INSTRUMENTAL conditions
        std::cout << "\n  INSTRUMENTAL conditions (" << action.commitments.instrumental.size() << "):" << std::endl;
        for (const auto& cond : action.commitments.instrumental) {
            std::cout << "    FOR clause: '" << cond.for_clause << "'" << std::endl;
            std::cout << "    Query: " << cond.sparql_query.substr(0, std::min(size_t(50), cond.sparql_query.size())) << "..." << std::endl;

            // Validate FOR clause is not empty for expected actions
            if (action.name == "GuideClient" || action.name == "MoveToLocation") {
                EXPECT_FALSE(cond.for_clause.empty()) << "FOR clause should not be empty for " << action.name;
                std::cout << "    ✓ FOR clause: " << cond.for_clause << std::endl;
            }
        }

        // Test ENGAGEMENT conditions
        std::cout << "\n  ENGAGEMENT conditions (" << action.commitments.engagement.size() << "):" << std::endl;
        for (const auto& cond : action.commitments.engagement) {
            std::cout << "    FOR clause: '" << cond.for_clause << "'" << std::endl;
            std::cout << "    Query: " << cond.sparql_query.substr(0, std::min(size_t(50), cond.sparql_query.size())) << "..." << std::endl;

            if (action.name == "GuideClient") {
                EXPECT_FALSE(cond.for_clause.empty()) << "FOR clause should not be empty for GuideClient engagement";
                EXPECT_TRUE(cond.for_clause.find("C") != std::string::npos) << "FOR clause should contain 'C' (parameter)";
                std::cout << "    ✓ FOR clause: " << cond.for_clause << std::endl;
            }
        }

        // Test COMMON_GROUND conditions
        std::cout << "\n  COMMON_GROUND conditions (" << action.commitments.common_ground.size() << "):" << std::endl;
        for (const auto& cond : action.commitments.common_ground) {
            std::cout << "    FOR clause: '" << cond.for_clause << "'" << std::endl;
            std::cout << "    Query: " << cond.sparql_query.substr(0, std::min(size_t(50), cond.sparql_query.size())) << "..." << std::endl;

            if (action.name == "GuideClient") {
                EXPECT_FALSE(cond.for_clause.empty()) << "FOR clause should not be empty for GuideClient common ground";
                EXPECT_TRUE(cond.for_clause.find("both") != std::string::npos) << "FOR clause should contain 'both'";
                std::cout << "    ✓ FOR clause: " << cond.for_clause << std::endl;
            }
        }
    }
}

} // namespace procedural

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
