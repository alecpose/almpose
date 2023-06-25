#ifndef FRAMEWORK_MARKERS_VISUALIZATION_HPP
#define FRAMEWORK_MARKERS_VISUALIZATION_HPP

#include <metavision/sdk/ui/utils/mt_window.h>
#include <metavision/sdk/ui/utils/window.h>
#include <metavision/sdk/ui/utils/event_loop.h>

#include <boost/thread.hpp>
#include <atomic>
#include <chrono>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "csvfile.h"
#include "buffers.hpp"
#include "types.hpp"

#include "../markers.hpp"
#include "../camera.hpp"
#include <algorithm>
#include "../logger.hpp"
#include "../runtime_manager.hpp"
#include "opencv2/aruco.hpp"

//class VisualizationProducer {
//public:
//    VisualizationProducer(int width, int height);
//    void registerBatch(EventBatch *batch);
//    void reset_state();
//    cv::Mat getOutput();
//
//private:
//    int width;
//    int height;
//
//    cv::Mat img;
//    cv::Mat output;
//
//};

class VisualizationController {
public:
    VisualizationController(int width, int height, Camera *cam, RuntimeManager *runtime);
    Buffers buffers;
    Camera *cam_obj;
    RuntimeManager *runtime_manager;

    std::shared_ptr<Logger> logger;
    void start();

    void getOutput();
    void resetState();

    bool isFinished();


//    VisualizationProducer *prod;
private:
    int width;
    int height;

    std::atomic<bool> to_close;
    std::atomic<bool> closed;


    Metavision::MTWindow *ui_ptr;

    void drawCurrentState();

    void runtimeLoop();

    float fps = 30;
    float accumulation_fps = 1000;

    Metavision::timestamp period = 0;
    Metavision::timestamp accumulation_period = 0;

    Metavision::timestamp next_output_required_at = 0;

    int cutoff = 0;
    int min_freq = 4000;

    cv::Mat img;
    cv::Mat labels;
    cv::Mat counts;
    cv::Mat output;

    boost::thread *thread_ptr;
};



#endif //FRAMEWORK_MARKERS_VISUALIZATION_HPP
