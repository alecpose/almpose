#ifndef FRAMEWORK_MARKERS_LOGGER_HPP
#define FRAMEWORK_MARKERS_LOGGER_HPP
#include "utils/types.hpp"
#include <opencv2/opencv.hpp>

#include "utils/csvfile.h"
#include <chrono>
#include <unistd.h>
#include <unordered_map>
#include <atomic>
class Logger {
public:
    Logger(bool csv_logging, std::string ros_tf_parent_name);
    void start();
    std::shared_ptr<OutputQueue> output_queue;
    std::shared_ptr<std::unordered_map<int, std::shared_ptr<OutputEntry>>> getCurrentStatus();

private:
    bool csv_logging_enabled;
    std::string ros_tf_parent;

    csvfile *csv;
    [[noreturn]] void loggerLoop();
    std::shared_ptr<std::jthread> thread_ptr;

    std::mutex current_outputs_mutex;
    std::shared_ptr<std::unordered_map<int, std::shared_ptr<OutputEntry>>> current_outputs_map;

    std::shared_ptr<OutputQueue> output_queue_ptr;


    std::atomic_bool record_status;
    int logged_outputs;

    std::shared_ptr<OutputEntry> current_object;

    void logToCsv(std::shared_ptr<OutputEntry> object);
};


#endif //FRAMEWORK_MARKERS_LOGGER_HPP
