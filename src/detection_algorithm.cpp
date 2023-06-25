#include "../include/detection_algorithm.hpp"

DetectionAlgorithm::DetectionAlgorithm(int width, int height, MarkersManager *markers, std::shared_ptr<Logger> logger) {
    camera_width = width;
    camera_height = height;
    markers_manager = markers;

    logger_ptr = logger;

    period = (long long)1000000 / fps;
    cutoff = min_freq / period;

    detector_busy = false;
    detection_timestamp = 0;
    new_detections_available = false;

    int size_hist[3] = { camera_height, camera_width, 20};
    gathered_timestamps = cv::Mat(3, size_hist, CV_32S, cv::Scalar::all(0));
    histogram = cv::Mat::zeros(camera_height, camera_width, CV_8UC1);

}

void DetectionAlgorithm::detectMarkers(std::shared_ptr<Deque> deque){
    bool detection_done = false;

    long long current_ts = (deque->end()-1)->get()->first->back()->t;
    long long finish_ts = current_ts - period;
    for (int i = int(deque->size()) - 1; i >= 0; i--) {
        for (const Metavision::Event2d* ev : *deque->at(i)->first) {
            if (histogram.at<uint8_t>(ev->y, ev->x) < 20){
                gathered_timestamps.at<int>(ev->y, ev->x, (histogram.at<uint8_t>(ev->y, ev->x))) = ev->t;
            }
            histogram.at<uint8_t>(ev->y, ev->x) += 1;


            if(ev->t < finish_ts){


                cv::Mat stats;
                cv::Mat centroids;
                std::vector<std::vector<cv::Point>> contours;
                std::vector<cv::Vec4i> hierarchy;
                std::vector<cv::Rect> boundRect;
                std::vector<int> areas;
                std::vector<double> frequencies;
                std::vector<std::vector<cv::Point>> filtered_contours;
                contours.clear();


                cv::Mat histogram_with_threshold;
                cv::threshold(histogram, histogram_with_threshold,  cutoff, 255,  cv::THRESH_BINARY);
                cv::findContours(histogram_with_threshold, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


                for (unsigned int j = 0; j < contours.size(); j++) {
                    double area = cv::contourArea(contours.at(j));

                    if(min_area < area){
                        filtered_contours.push_back(contours.at(j));
                        boundRect.push_back(boundingRect( contours.at(j) ));
                        areas.push_back(area);
                    }
                }


                for (unsigned int j = 0; j < boundRect.size(); j++) {

                    std::vector<int> diffs(201, 0);

                    for (int x = boundRect.at(j).x; x <= boundRect.at(j).x + boundRect.at(j).width; ++x){
                        for (int y = boundRect.at(j).y; y <= boundRect.at(j).y + boundRect.at(j).height; ++y){

                            for (int channel = 0; channel < std::max(int(histogram.at<uint8_t>(y, x)) - 1, 0); ++channel){
                                diffs.at(std::min(std::abs(gathered_timestamps.at<int>(y, x, channel+1) - gathered_timestamps.at<int>(y, x, channel)), 200)) += 1;
                            }
                        }
                    }
                    diffs.back() = 0;

                    auto it = std::max_element(std::begin(diffs), std::end(diffs));
                    auto period_mode = std::distance(std::begin(diffs), it);
                    if (period_mode > 0){
                        auto sum = double((period_mode * diffs.at(period_mode) + ((period_mode-1) * diffs.at(period_mode-1)) + ((period_mode+1) * diffs.at(period_mode+1))));
                        auto cnt = double(diffs.at(period_mode) + diffs.at(period_mode+1) + diffs.at(period_mode - 1));
                        auto freq = double(sum / cnt);
                        frequencies.push_back(double(1000000 / freq));
                    }
                    else{
                        frequencies.push_back(double(0.0));
                    }
                }
                assigned_contours = filtered_contours;
                assigned_frequencies = frequencies;

                detection_done = true;
                detection_timestamp = current_ts;
                new_detections_available = true;




            }
            if (detection_done)
            {
                break;
            }
        }
        if (detection_done)
        {
            break;
        }
    }
//    std::cout << "Detection done" << std::endl;
    detector_busy = false;
    histogram = cv::Mat::zeros(camera_height, camera_width, CV_8UC1);

}

[[noreturn]] void DetectionAlgorithm::runtimeLoop() {
    new_detection_deque = std::make_shared<Deque>();
    initial_tracking_buffer_deque = std::make_shared<Deque>();
    while(true){

        if (new_detections_available.load()){
            markers_manager->registerDetections(assigned_frequencies,
                                                assigned_contours,
                                                detection_timestamp);
            markers_manager->spawnMarkers(initial_tracking_buffer_deque, &buffers);

            new_detections_available = false;
            initial_tracking_buffer_deque = std::make_shared<Deque>();

        }

        if(buffers.getBatchAndSendFurther()){
            initial_tracking_buffer_deque->push_back(buffers.current_batch);
            new_detection_deque->push_back(buffers.current_batch);
            length_of_current_buffer += int(buffers.current_batch->first->back()->t - buffers.current_batch->first->front()->t);
        }

        if(!new_detections_available.load() && !detector_busy.load() && length_of_current_buffer > period){

            detector_busy = true;
            detect_thread_ptr = std::jthread(&DetectionAlgorithm::detectMarkers, this, new_detection_deque);
//            initial_tracking_buffer_deque = std::make_shared<Deque>();
            new_detection_deque = std::make_shared<Deque>();
            length_of_current_buffer = 0;
        }
    }
}

void DetectionAlgorithm::start() {
    if(buffers.input_buffer != nullptr){
        thread_ptr = std::jthread(&DetectionAlgorithm::runtimeLoop, this);
    }
}
