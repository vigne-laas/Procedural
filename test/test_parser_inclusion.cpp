#include <gtest/gtest.h>
#include <filesystem>
#include <memory>

#include "procedural/memory/FullParser.h"
#include "procedural/memory/ProceduralFullReader.h"

using namespace procedural;

class ParserInclusionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up test data directory
        test_data_dir = std::filesystem::current_path() / "src" / "Procedural" / "test" / "data";

        // Ensure test data directory exists
        if (!std::filesystem::exists(test_data_dir)) {
            GTEST_SKIP() << "Test data directory not found: " << test_data_dir;
        }

        parser = std::make_unique<FullParser>();
    }

    void TearDown() override {
        parser.reset();
    }

    std::filesystem::path test_data_dir;
    std::unique_ptr<FullParser> parser;

    // Helper method to find an action by name
    const Action_t* findAction(const Actions_t& actions, const std::string& name) {
        for (const auto& action : actions.actions) {
            if (action.name == name) {
                return &action;
            }
        }
        return nullptr;
    }

    // Helper method to find a practice by name
    const Practice* findPractice(const std::vector<Practice*>& practices, const std::string& name) {
        for (const auto& practice : practices) {
            if (practice->name == name) {
                return practice;
            }
        }
        return nullptr;
    }

    // Helper method to find a task by name
    const Abstract_task_t* findTask(const std::vector<Abstract_task_t>& tasks, const std::string& name) {
        for (const auto& task : tasks) {
            if (task.name == name) {
                return &task;
            }
        }
        return nullptr;
    }
};

TEST_F(ParserInclusionTest, BasicInclusion) {
    std::string main_file = test_data_dir / "main_test.hatp";

    // Test that the file exists
    ASSERT_TRUE(std::filesystem::exists(main_file)) << "Main test file not found: " << main_file;

    // Parse the file with inclusions
    parser->setCurrentDirectory(test_data_dir.string());

    try {
        parser->parseFileWithInclusions(main_file);

        // Verify that actions were loaded and merged
        auto actions = parser->getActions();
        ASSERT_GT(actions.actions.size(), 0) << "No actions were parsed";

        // Find the Move action which should be merged from multiple files
        const Action_t* move_action = findAction(actions, "Move");
        ASSERT_NE(move_action, nullptr) << "Move action not found";

        // Verify the action has elements from multiple files
        EXPECT_GT(move_action->preconditions.size(), 0) << "Move action should have preconditions";
        EXPECT_GT(move_action->effects.size(), 0) << "Move action should have effects";

        // Verify practices were loaded
        auto practices = parser->getPractices();
        ASSERT_GT(practices.size(), 0) << "No practices were parsed";

        const Practice* greet_practice = findPractice(practices, "Greet");
        ASSERT_NE(greet_practice, nullptr) << "Greet practice not found";

        // Verify practice was merged - should have competences from multiple files
        EXPECT_GT(greet_practice->competences.size(), 2) << "Greet practice should have merged competences";

        std::cout << "=== TEST RESULTS ===" << std::endl;
        std::cout << "Total actions parsed: " << actions.actions.size() << std::endl;
        std::cout << "Total practices parsed: " << practices.size() << std::endl;

    } catch (const std::exception& e) {
        FAIL() << "Exception during parsing: " << e.what();
    }
}

TEST_F(ParserInclusionTest, CircularInclusionDetection) {
    std::string circular_file = test_data_dir / "circular_a.hatp";

    // Test that the file exists
    ASSERT_TRUE(std::filesystem::exists(circular_file)) << "Circular test file not found: " << circular_file;

    parser->setCurrentDirectory(test_data_dir.string());

    // This should throw an exception due to circular dependency
    EXPECT_THROW({
        parser->parseFileWithInclusions(circular_file);
    }, std::runtime_error) << "Circular dependency should be detected";
}

TEST_F(ParserInclusionTest, ElementMerging) {
    std::string main_file = test_data_dir / "main_test.hatp";

    ASSERT_TRUE(std::filesystem::exists(main_file));

    parser->setCurrentDirectory(test_data_dir.string());

    try {
        parser->parseFileWithInclusions(main_file);

        auto actions = parser->getActions();

        // Test Move action merging
        const Action_t* move_action = findAction(actions, "Move");
        ASSERT_NE(move_action, nullptr);

        // Should have preconditions from basic_actions.hatp
        EXPECT_GT(move_action->preconditions.size(), 0);

        // Should have effects from basic_actions.hatp
        EXPECT_GT(move_action->effects.size(), 0);

        // Should have execution from robot_actions.hatp
        EXPECT_GT(move_action->executions_bloc.size(), 0);

        // Should have duration from basic_actions.hatp
        EXPECT_GT(move_action->duration, 0);

        // Should have description from main_test.hatp (latest definition)
        EXPECT_GT(move_action->description.description.size(), 0);

        std::cout << "Move action merge verification:" << std::endl;
        std::cout << "  Preconditions: " << move_action->preconditions.size() << std::endl;
        std::cout << "  Effects: " << move_action->effects.size() << std::endl;
        std::cout << "  Execution actions: " << move_action->executions_bloc.size() << std::endl;
        std::cout << "  Duration: " << move_action->duration << std::endl;
        std::cout << "  Description triplets: " << move_action->description.description.size() << std::endl;

    } catch (const std::exception& e) {
        FAIL() << "Exception during parsing: " << e.what();
    }
}

TEST_F(ParserInclusionTest, PracticeMerging) {
    std::string main_file = test_data_dir / "main_test.hatp";

    ASSERT_TRUE(std::filesystem::exists(main_file));

    parser->setCurrentDirectory(test_data_dir.string());

    try {
        parser->parseFileWithInclusions(main_file);

        auto practices = parser->getPractices();
        const Practice* greet_practice = findPractice(practices, "Greet");
        ASSERT_NE(greet_practice, nullptr);

        // Should have competences from both files
        EXPECT_GE(greet_practice->competences.size(), 3); // At least 3 competences

        // Should have rules from main_test.hatp
        EXPECT_GT(greet_practice->rules.size(), 0);

        // Check specific competences
        bool has_social = false, has_speech = false, has_gesture = false;
        for (const auto& comp : greet_practice->competences) {
            if (comp == "social_interaction") has_social = true;
            if (comp == "speech_synthesis") has_speech = true;
            if (comp == "gesture_recognition") has_gesture = true;
        }

        EXPECT_TRUE(has_social) << "Should have social_interaction competence";
        EXPECT_TRUE(has_speech) << "Should have speech_synthesis competence";
        EXPECT_TRUE(has_gesture) << "Should have gesture_recognition competence";

        std::cout << "Greet practice merge verification:" << std::endl;
        std::cout << "  Total competences: " << greet_practice->competences.size() << std::endl;
        std::cout << "  Rules: " << greet_practice->rules.size() << std::endl;

    } catch (const std::exception& e) {
        FAIL() << "Exception during parsing: " << e.what();
    }
}

TEST_F(ParserInclusionTest, MultipleFileTypes) {
    std::string main_file = test_data_dir / "main_test.hatp";

    ASSERT_TRUE(std::filesystem::exists(main_file));

    parser->setCurrentDirectory(test_data_dir.string());

    try {
        parser->parseFileWithInclusions(main_file);

        auto actions = parser->getActions();
        auto practices = parser->getPractices();
        auto tasks = parser->getTasks();
        auto priorities = parser->getPriorities();

        // Verify we have elements from all file types
        EXPECT_GT(actions.actions.size(), 0) << "Should have actions";
        EXPECT_GT(practices.size(), 0) << "Should have practices";
        EXPECT_GT(tasks.size(), 0) << "Should have tasks";
        EXPECT_GT(priorities.size(), 0) << "Should have priorities";

        // Verify specific actions exist
        EXPECT_NE(findAction(actions, "Move"), nullptr);
        EXPECT_NE(findAction(actions, "PickUp"), nullptr);
        EXPECT_NE(findAction(actions, "PutDown"), nullptr);

        // Verify specific practices exist
        EXPECT_NE(findPractice(practices, "Greet"), nullptr);
        EXPECT_NE(findPractice(practices, "Assistance"), nullptr);

        // Verify specific tasks exist
        EXPECT_NE(findTask(tasks, "transport"), nullptr);
        EXPECT_NE(findTask(tasks, "deliver"), nullptr);

        std::cout << "Multi-file parsing summary:" << std::endl;
        std::cout << "  Actions: " << actions.actions.size() << std::endl;
        std::cout << "  Practices: " << practices.size() << std::endl;
        std::cout << "  Tasks: " << tasks.size() << std::endl;
        std::cout << "  Priorities: " << priorities.size() << std::endl;

    } catch (const std::exception& e) {
        FAIL() << "Exception during parsing: " << e.what();
    }
}

// Main function for running tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}