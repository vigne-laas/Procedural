#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "procedural/memory/FullParser.h"
#include "procedural/memory/ProceduralFullReader.h"

namespace procedural {

class FullParserCommitmentsTest : public ::testing::Test {
protected:
    void SetUp() override {
        package_path = ros::package::getPath("procedural");
    }

    std::string package_path;
};

TEST_F(FullParserCommitmentsTest, testCommitmentsParsingWithForClause)
{
    // Create reader and parse domain with FOR clauses
    ProceduralFullReader reader;
    std::string file_path = package_path + "/test/test_commitment_with_for.dom";
    bool readResult = reader.read(file_path);

    ASSERT_TRUE(readResult) << "Failed to read commitment test domain with FOR clauses";

    // Get the actions
    Actions_t actions = reader.getActions();
    ASSERT_GT(actions.actions.size(), 0) << "No actions parsed from domain";

    // Find the TestCommitmentWithFor action
    Action_t* test_action = nullptr;
    for (auto& action : actions.actions)
    {
        if (action.name == "TestCommitmentWithFor")
        {
            test_action = &action;
            break;
        }
    }

    ASSERT_NE(test_action, nullptr) << "TestCommitmentWithFor action not found";

    // Check that commitments were parsed
    ASSERT_TRUE(test_action->has_commitments) << "Commitments not detected";
    ASSERT_NE(test_action->commitments, nullptr) << "Commitments pointer is null";

    // Check INSTRUMENTAL block - should have 2 conditions
    ASSERT_EQ(test_action->commitments->instrumental.size(), 2)
        << "Expected 2 instrumental conditions";

    // Check first instrumental condition: "Battery sufficient" FOR robot
    const auto& inst1 = test_action->commitments->instrumental[0];
    EXPECT_EQ(inst1.description, "Battery sufficient")
        << "First instrumental description should be 'Battery sufficient'";
    EXPECT_EQ(inst1.for_clause, "robot")
        << "First instrumental for_clause should be 'robot'";
    EXPECT_TRUE(inst1.sparql_query.find("hasBatteryLevel") != std::string::npos)
        << "First instrumental condition should contain hasBatteryLevel";

    // Check second instrumental condition: "Path is clear" FOR environment
    const auto& inst2 = test_action->commitments->instrumental[1];
    EXPECT_EQ(inst2.description, "Path is clear")
        << "Second instrumental description should be 'Path is clear'";
    EXPECT_EQ(inst2.for_clause, "environment")
        << "Second instrumental for_clause should be 'environment'";
    EXPECT_TRUE(inst2.sparql_query.find("blockingPath") != std::string::npos)
        << "Second instrumental condition should contain blockingPath";

    // Check ENGAGEMENT block - should have 1 condition
    ASSERT_EQ(test_action->commitments->engagement.size(), 1)
        << "Expected 1 engagement condition";

    // Check engagement condition: "Client follows" FOR ?C
    const auto& eng1 = test_action->commitments->engagement[0];
    EXPECT_EQ(eng1.description, "Client follows")
        << "Engagement description should be 'Client follows'";
    EXPECT_EQ(eng1.for_clause, "?C")
        << "Engagement for_clause should be '?C'";
    EXPECT_TRUE(eng1.sparql_query.find("isFollowing") != std::string::npos)
        << "Engagement condition should contain isFollowing";

    // Check COMMON_GROUND block - should have 1 condition
    ASSERT_EQ(test_action->commitments->common_ground.size(), 1)
        << "Expected 1 common ground condition";

    // Check common ground condition: "Mutual goal understanding" FOR both(robot, ?C)
    const auto& cg1 = test_action->commitments->common_ground[0];
    EXPECT_EQ(cg1.description, "Mutual goal understanding")
        << "Common ground description should be 'Mutual goal understanding'";
    EXPECT_TRUE(cg1.for_clause.find("both") != std::string::npos)
        << "Common ground for_clause should contain 'both'";
    EXPECT_TRUE(cg1.sparql_query.find("hasGoal") != std::string::npos)
        << "Common ground condition should contain hasGoal";

    // Check recovery actions
    EXPECT_EQ(test_action->commitments->on_instrumental_failure, "stop_and_diagnose")
        << "Instrumental failure reaction should be stop_and_diagnose";
    EXPECT_EQ(test_action->commitments->on_engagement_failure, "call_client_back")
        << "Engagement failure reaction should be call_client_back";
    EXPECT_EQ(test_action->commitments->on_common_ground_failure, "clarify_goal")
        << "Common ground failure reaction should be clarify_goal";

    // Check recovery strategy
    EXPECT_EQ(test_action->commitments->recovery_strategy.mode, "retry")
        << "Recovery mode should be retry";
    EXPECT_EQ(test_action->commitments->recovery_strategy.max_attempts, 3)
        << "Max attempts should be 3";
    EXPECT_DOUBLE_EQ(test_action->commitments->recovery_strategy.timeout, 30.0)
        << "Timeout should be 30.0";

    std::cout << "\n=== Parsed Action with FOR Clauses ===" << std::endl;
    std::cout << "Action: " << test_action->name << std::endl;
    std::cout << "Has commitments: " << test_action->has_commitments << std::endl;
    std::cout << "Instrumental conditions: " << test_action->commitments->instrumental.size() << std::endl;
    for (size_t i = 0; i < test_action->commitments->instrumental.size(); ++i) {
        const auto& cond = test_action->commitments->instrumental[i];
        std::cout << "  [" << i << "] \"" << cond.description << "\" FOR " << cond.for_clause << std::endl;
    }
    std::cout << "Engagement conditions: " << test_action->commitments->engagement.size() << std::endl;
    for (size_t i = 0; i < test_action->commitments->engagement.size(); ++i) {
        const auto& cond = test_action->commitments->engagement[i];
        std::cout << "  [" << i << "] \"" << cond.description << "\" FOR " << cond.for_clause << std::endl;
    }
    std::cout << "=======================================\n" << std::endl;
}

TEST_F(FullParserCommitmentsTest, testCommitmentsWithoutForClause)
{
    ProceduralFullReader reader;
    std::string file_path = package_path + "/test/test_commitment_with_for.dom";
    bool readResult = reader.read(file_path);

    ASSERT_TRUE(readResult);

    Actions_t actions = reader.getActions();

    // Find the TestNoForClause action
    Action_t* test_action = nullptr;
    for (auto& action : actions.actions)
    {
        if (action.name == "TestNoForClause")
        {
            test_action = &action;
            break;
        }
    }

    ASSERT_NE(test_action, nullptr) << "TestNoForClause action not found";
    ASSERT_TRUE(test_action->has_commitments) << "Commitments should be detected";
    ASSERT_NE(test_action->commitments, nullptr);

    // Check instrumental condition without FOR clause
    ASSERT_EQ(test_action->commitments->instrumental.size(), 1);
    const auto& inst1 = test_action->commitments->instrumental[0];
    EXPECT_EQ(inst1.description, "System operational");
    EXPECT_EQ(inst1.for_clause, "")
        << "for_clause should be empty when not specified";
    EXPECT_TRUE(inst1.sparql_query.find("hasStatus") != std::string::npos);

    // Check recovery action
    EXPECT_EQ(test_action->commitments->on_instrumental_failure, "restart_system");
}

TEST_F(FullParserCommitmentsTest, testActionWithoutCommitments)
{
    ProceduralFullReader reader;
    std::string file_path = package_path + "/test/test_commitment_with_for.dom";
    bool readResult = reader.read(file_path);

    ASSERT_TRUE(readResult);

    Actions_t actions = reader.getActions();

    // Find a recovery action (should not have commitments)
    Action_t* recovery_action = nullptr;
    for (auto& action : actions.actions)
    {
        if (action.name == "stop_and_diagnose")
        {
            recovery_action = &action;
            break;
        }
    }

    ASSERT_NE(recovery_action, nullptr);
    EXPECT_FALSE(recovery_action->has_commitments)
        << "Recovery actions should not have commitments";
}

} // namespace procedural

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_fullparser_commitments");
    return RUN_ALL_TESTS();
}
