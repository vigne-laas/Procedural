#include <gtest/gtest.h>
#include <filesystem>
#include <memory>
#include <algorithm>

#include "procedural/memory/FullParser.h"

using namespace procedural;

// ANTLR lexer captures single spaces as SPACE tokens (not skipped),
// which can end up prepended to parsed names. Trim for comparison.
static std::string trimName(const std::string& s) {
    auto start = s.find_first_not_of(' ');
    if (start == std::string::npos) return "";
    auto end = s.find_last_not_of(' ');
    return s.substr(start, end - start + 1);
}

class HospitalDomainTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Try catkin workspace path first, then current_path
        std::vector<std::filesystem::path> candidates = {
            "/home/avigne/Projets/ArchiThese/catkin_ws/src/hospital_resources/domaines/hospital_domain.dom",
            std::filesystem::current_path() / "src" / "hospital_resources" / "domaines" / "hospital_domain.dom",
            std::filesystem::current_path().parent_path() / "src" / "hospital_resources" / "domaines" / "hospital_domain.dom",
        };
        for (const auto& c : candidates) {
            if (std::filesystem::exists(c)) {
                hospital_dom = c;
                break;
            }
        }
        if (!std::filesystem::exists(hospital_dom)) {
            GTEST_SKIP() << "Hospital domain file not found in any candidate path";
        }
        parser = std::make_unique<FullParser>();
    }

    void TearDown() override {
        parser.reset();
    }

    std::filesystem::path hospital_dom;
    std::unique_ptr<FullParser> parser;
};

TEST_F(HospitalDomainTest, ParseWithoutError) {
    parser->setCurrentDirectory(hospital_dom.parent_path().string());
    ASSERT_NO_THROW({
        parser->parseFileWithInclusions(hospital_dom.string());
    }) << "Hospital domain file should parse without errors";
    std::cout << "PARSE OK: No exceptions thrown" << std::endl;
}

TEST_F(HospitalDomainTest, ActionsCount) {
    parser->setCurrentDirectory(hospital_dom.parent_path().string());
    parser->parseFileWithInclusions(hospital_dom.string());
    auto actions = parser->getActions();
    std::cout << "Actions parsed: " << actions.actions.size() << std::endl;
    for (const auto& a : actions.actions) {
        std::cout << "  - " << a.name << " (" << a.arguments.size() << " args)" << std::endl;
    }
    EXPECT_EQ(actions.actions.size(), 10) << "Expected 10 actions";
}

TEST_F(HospitalDomainTest, PrioritiesCount) {
    parser->setCurrentDirectory(hospital_dom.parent_path().string());
    parser->parseFileWithInclusions(hospital_dom.string());
    auto priorities = parser->getPriorities();
    std::cout << "Priorities parsed: " << priorities.size() << std::endl;
    for (const auto& p : priorities) {
        std::cout << "  - " << p->name << " (level=" << p->level << ")" << std::endl;
    }
    EXPECT_EQ(priorities.size(), 6) << "Expected 6 priorities";
}

TEST_F(HospitalDomainTest, PriorityLevels) {
    parser->setCurrentDirectory(hospital_dom.parent_path().string());
    parser->parseFileWithInclusions(hospital_dom.string());
    auto priorities = parser->getPriorities();

    std::map<std::string, int> expected = {
        {"PrepareInsulinDelivery", 4},
        {"RespondToFall", 8},
        {"RespondToHypoglycemia", 10},
        {"ReassurePatient", 6},
        {"TransmitTaskInfo", 6},
        {"ReportStatus", 6},
    };

    for (const auto& p : priorities) {
        std::string name = trimName(p->name);
        auto it = expected.find(name);
        ASSERT_NE(it, expected.end()) << "Unexpected priority: [" << p->name << "]";
        EXPECT_EQ(p->level, it->second) << "Priority " << name << " has wrong level";
    }
}

TEST_F(HospitalDomainTest, TasksCount) {
    parser->setCurrentDirectory(hospital_dom.parent_path().string());
    parser->parseFileWithInclusions(hospital_dom.string());
    auto tasks = parser->getTasks();
    std::cout << "Tasks parsed: " << tasks.size() << std::endl;
    for (const auto& t : tasks) {
        std::cout << "  - " << t.name << " (" << t.methods_.size() << " methods)" << std::endl;
    }
    EXPECT_EQ(tasks.size(), 4) << "Expected 4 tasks";
}

TEST_F(HospitalDomainTest, CommuniquerStatutMethods) {
    parser->setCurrentDirectory(hospital_dom.parent_path().string());
    parser->parseFileWithInclusions(hospital_dom.string());
    auto tasks = parser->getTasks();

    bool found = false;
    for (const auto& t : tasks) {
        if (trimName(t.name) == "CommuniquerStatut") {
            found = true;
            EXPECT_EQ(t.methods_.size(), 3) << "CommuniquerStatut should have 3 methods";
            std::cout << "CommuniquerStatut methods:" << std::endl;
            for (const auto& m : t.methods_) {
                std::cout << "  - " << trimName(m.name) << " (" << m.preconditions.size() << " preconditions)" << std::endl;
            }
        }
    }
    EXPECT_TRUE(found) << "CommuniquerStatut task not found";
}

TEST_F(HospitalDomainTest, PracticeFramesCount) {
    parser->setCurrentDirectory(hospital_dom.parent_path().string());
    parser->parseFileWithInclusions(hospital_dom.string());
    auto frames = parser->getPracticeFrames();
    std::cout << "Practice frames parsed: " << frames.size() << std::endl;
    for (const auto& f : frames) {
        std::cout << "  - " << f->name << std::endl;
    }
    EXPECT_EQ(frames.size(), 1) << "Expected 1 practice frame (Hospital)";
}

TEST_F(HospitalDomainTest, PracticesCount) {
    parser->setCurrentDirectory(hospital_dom.parent_path().string());
    parser->parseFileWithInclusions(hospital_dom.string());
    auto practices = parser->getPractices();
    std::cout << "Practices parsed: " << practices.size() << std::endl;
    for (const auto& p : practices) {
        std::cout << "  - " << p->name << std::endl;
    }
    EXPECT_EQ(practices.size(), 4) << "Expected 4 practices";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
