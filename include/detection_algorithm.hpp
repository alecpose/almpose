#ifndef FRAMEWORK_MARKERS_DETECTION_ALGORITHM_HPP
#define FRAMEWORK_MARKERS_DETECTION_ALGORITHM_HPP

#include <opencv2/opencv.hpp>
#include "utils/buffers.hpp"
#include <boost/thread.hpp>
#include <atomic>
#include <chrono>
#include <thread>
#include "markers.hpp"
#include "logger.hpp"

class DetectionAlgorithm {
public:
    DetectionAlgorithm(int width, int height, MarkersManager *markers, std::shared_ptr<Logger> logger);

    void start();
    Buffers buffers;

private:
    std::shared_ptr<Logger> logger_ptr;
    int camera_width;
    int camera_height;

    [[noreturn]] void runtimeLoop();

    cv::Mat img;

    std::shared_ptr<Deque> new_detection_deque = nullptr;
    std::shared_ptr<Deque> initial_tracking_buffer_deque = nullptr;

    MarkersManager *markers_manager;

    std::atomic<bool> detector_busy;
    std::atomic<bool> new_detections_available;

    void detectMarkers(std::shared_ptr<Deque> deque);

    int length_of_current_buffer = 0;

    double fps = 1000;
    Metavision::timestamp period = 0;
    Metavision::timestamp next_output_required_at = 0;

    std::jthread thread_ptr;
    std::jthread detect_thread_ptr;



    //detection variables
    cv::Mat gathered_timestamps;
    cv::Mat histogram;

    std::vector<std::vector<cv::Point>> assigned_contours;
    std::vector<double> assigned_frequencies;
    std::atomic<long long> detection_timestamp;



    int min_freq = 2000;
    double min_area = 3;
    int cutoff;

};


#endif //FRAMEWORK_MARKERS_DETECTION_ALGORITHM_HPP
