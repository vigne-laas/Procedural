#include <ros/ros.h>
#include <signal.h>

#include "procedural/task_recognition/Reader/MemoryDomainReader.h"
#include "procedural/task_recognition/Builder/HTNBuilder.h"
#include "procedural/utils/Logger.h"

using namespace procedural;

class TaskRecognitionNode {
public:
    TaskRecognitionNode() : nh_("~"), htn_builder_(nullptr) {
        // Initialize parameters
        nh_.param<std::string>("memory_service_namespace", memory_service_namespace_, "procedural_memory");
        nh_.param<std::string>("task_filter", task_filter_, "");

        LOG_INFO << "Task Recognition Node starting with parameters:";
        LOG_INFO << "  Memory service namespace: " << memory_service_namespace_;
        LOG_INFO << "  Task filter: " << (task_filter_.empty() ? "none" : task_filter_);

        initializeTaskRecognition();
    }

    ~TaskRecognitionNode() {
        if (htn_builder_) {
            delete htn_builder_;
        }
    }

    bool initializeTaskRecognition() {
        try {
            // Create memory domain reader - explicitly use the service namespace constructor
            auto memory_reader = std::unique_ptr<MemoryDomainReader>(
                new MemoryDomainReader(nh_, memory_service_namespace_));

            // Wait for memory service
            LOG_INFO << "Waiting for memory service to become available...";
            if (!memory_reader->waitForService(ros::Duration(30.0))) {
                LOG_ERROR << "Memory service not available after 30 seconds";
                return false;
            }

            // Read tasks from memory
            LOG_INFO << "Reading tasks from memory service...";
            if (!memory_reader->read(task_filter_)) {
                LOG_ERROR << "Failed to read tasks from memory service";
                return false;
            }

            LOG_INFO << "Successfully loaded " << memory_reader->getTaskCount() << " tasks";

            // Build HTN using HTNBuilder
            LOG_INFO << "Building HTN structure...";
            auto htn = memory_reader->getHTN();

            htn_builder_ = new HTNBuilder();
            if (!htn_builder_->build(htn)) {
                LOG_ERROR << "Failed to build HTN structure";
                return false;
            }

            auto tasks = htn_builder_->getTasks();
            LOG_INFO << "Successfully built HTN with " << tasks.size() << " tasks";

            LOG_INFO << "Task recognition system initialized successfully";
            return true;

        } catch (const std::exception& e) {
            LOG_ERROR << "Exception during initialization: " << e.what();
            return false;
        }
    }

    void processTaskRecognition() {
        // This is a placeholder for task recognition processing
        // You would typically implement HTN planning/recognition logic here

        if (!htn_builder_) {
            return;
        }

        auto tasks = htn_builder_->getTasks();
        // Process tasks for recognition, planning, etc.
    }

    void spin() {
        if (!htn_builder_) {
            LOG_ERROR << "HTN builder not initialized, cannot spin";
            return;
        }

        ros::Rate rate(1); // 1 Hz - tasks typically don't need high frequency processing

        while (ros::ok()) {
            // Process any incoming ROS messages
            ros::spinOnce();

            // Process task recognition
            processTaskRecognition();

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    HTNBuilder* htn_builder_;

    // Parameters
    std::string memory_service_namespace_;
    std::string task_filter_;
};

void signalHandler(int sig) {
    LOG_INFO << "Shutting down task recognition node...";
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_recognition_node");

    // Set up signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        TaskRecognitionNode node;
        node.spin();
    } catch (const std::exception& e) {
        LOG_ERROR << "Task recognition node failed: " << e.what();
        return 1;
    }

    LOG_INFO << "Task recognition node shutdown complete";
    return 0;
}