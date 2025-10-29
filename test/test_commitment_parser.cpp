#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "procedural/task_recognition/Reader/DomainReader.h"
#include "procedural/utils/Logger.h"

namespace procedural {

TEST(CommitmentParserTest, testCommitmentsParsing)
{
    // Create an instance of DomainReader
    DomainReader domainReader;

    // Call the read method with the commitment test file
    std::string package_path = ros::package::getPath("procedural");
    std::string file_path = package_path + "/test/commitment_test.dom";
    bool readResult = domainReader.read(file_path);

    // Check the result of the read method
    ASSERT_TRUE(readResult) << "Failed to read commitment test domain";

    // Get the actions
    std::vector<PrimitiveActionParsed_t> actions = domainReader.getActions();

    // We should have 4 actions (TestSimpleCommitment + 3 reaction actions)
    ASSERT_EQ(actions.size(), 4) << "Expected 4 actions in commitment test domain";

    // Find the TestSimpleCommitment action
    PrimitiveActionParsed_t* test_action = nullptr;
    for (auto& action : actions)
    {
        if (action.name == "TestSimpleCommitment")
        {
            test_action = &action;
            break;
        }
    }

    ASSERT_NE(test_action, nullptr) << "TestSimpleCommitment action not found";

    // Check that commitments were parsed
    ASSERT_TRUE(test_action->commitments.has_commitments) << "Commitments not detected";

    // Check INSTRUMENTAL block
    ASSERT_EQ(test_action->commitments.instrumental.size(), 1)
        << "Expected 1 instrumental condition";
    EXPECT_TRUE(test_action->commitments.instrumental[0].sparql_query.find("hasBlockageStatus") != std::string::npos)
        << "Instrumental condition should contain hasBlockageStatus";

    // Check ENGAGEMENT block
    ASSERT_EQ(test_action->commitments.engagement.size(), 1)
        << "Expected 1 engagement condition";
    EXPECT_TRUE(test_action->commitments.engagement[0].sparql_query.find("hasProximityStatus") != std::string::npos)
        << "Engagement condition should contain hasProximityStatus";

    // Check COMMON_GROUND block
    ASSERT_EQ(test_action->commitments.common_ground.size(), 1)
        << "Expected 1 common ground condition";
    EXPECT_TRUE(test_action->commitments.common_ground[0].sparql_query.find("isVisibleTo") != std::string::npos)
        << "Common ground condition should contain isVisibleTo";

    // Check reaction mappings
    EXPECT_EQ(test_action->commitments.on_instrumental_failure, "stop_and_wait")
        << "Instrumental failure reaction should be stop_and_wait";
    EXPECT_EQ(test_action->commitments.on_engagement_failure, "turn_and_wait")
        << "Engagement failure reaction should be turn_and_wait";
    EXPECT_EQ(test_action->commitments.on_common_ground_failure, "clarify_goal")
        << "Common ground failure reaction should be clarify_goal";

    // Check recovery strategy
    EXPECT_EQ(test_action->commitments.recovery_strategy.mode, "continue")
        << "Recovery mode should be continue";
    EXPECT_EQ(test_action->commitments.recovery_strategy.max_attempts, 3)
        << "Max attempts should be 3";
    EXPECT_DOUBLE_EQ(test_action->commitments.recovery_strategy.timeout, 30.0)
        << "Timeout should be 30.0";

    // Display parsed action for debugging
    std::cout << "\n=== Parsed Action with Commitments ===" << std::endl;
    std::cout << *test_action << std::endl;
    std::cout << "======================================\n" << std::endl;
}

TEST(CommitmentParserTest, testActionWithoutCommitments)
{
    DomainReader domainReader;

    std::string package_path = ros::package::getPath("procedural");
    std::string file_path = package_path + "/test/commitment_test.dom";
    bool readResult = domainReader.read(file_path);

    ASSERT_TRUE(readResult);

    std::vector<PrimitiveActionParsed_t> actions = domainReader.getActions();

    // Find a reaction action (should not have commitments)
    PrimitiveActionParsed_t* reaction_action = nullptr;
    for (auto& action : actions)
    {
        if (action.name == "stop_and_wait")
        {
            reaction_action = &action;
            break;
        }
    }

    ASSERT_NE(reaction_action, nullptr) << "stop_and_wait action not found";

    // This action should NOT have commitments
    EXPECT_FALSE(reaction_action->commitments.has_commitments)
        << "Reaction actions should not have commitments";
}

} // namespace procedural

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_commitment_parser");
    return RUN_ALL_TESTS();
}
