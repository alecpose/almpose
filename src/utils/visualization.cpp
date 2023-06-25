#include "../../include/utils/visualization.hpp"

template<typename type>
std::vector<type> unique(cv::Mat in) {
    assert(in.channels() == 1 && "This implementation is only for single-channel images");
    auto begin = in.begin<type>(), end = in.end<type>();
    auto last = std::unique(begin, end);    // remove adjacent duplicates to reduce size
    std::sort(begin, last);                 // sort remaining elements
    last = std::unique(begin, last);        // remove duplicates
    return std::vector<type>(begin, last);
}

VisualizationController::VisualizationController(int width_new, int height_new, Camera *cam, RuntimeManager *runtime) {
    width = width_new;
    height = height_new;

    cam_obj = cam;
    runtime_manager = runtime;


    ui_ptr = new Metavision::MTWindow("Active markers", width, height, Metavision::Window::RenderMode::BGR);
    ui_ptr->set_keyboard_callback([this](Metavision::UIKeyEvent key, int scancode, Metavision::UIAction action, int mods) {
        if (action == Metavision::UIAction::RELEASE && key == Metavision::UIKeyEvent::KEY_ESCAPE){
            to_close = true;
            return;
        }
        if (action == Metavision::UIAction::RELEASE && key == Metavision::UIKeyEvent::KEY_P){
            std::cout << "PAUSE" << std::endl;
            return;
        }

    });
    img = cv::Mat::zeros(height, width, CV_8UC1);
    output = cv::Mat::zeros(height, width, CV_8UC3);
    labels = cv::Mat::zeros(height, width, CV_8UC1);
    counts = cv::Mat::zeros(height, width, CV_8UC1);

    to_close = false;
    closed = false;

    period = (long long)1000000 / fps;
    accumulation_period = (long long)1000000 / accumulation_fps;
    cutoff = min_freq / accumulation_period;

}

void VisualizationController::runtimeLoop() {
    int i = 0;
    bool to_check = false;

    logger = runtime_manager->getLogger();


    while(!to_close.load()){


        if (buffers.getBatch()){
            if (next_output_required_at == 0){
                next_output_required_at = buffers.current_batch->first->front()->t + period;
                std::cout << "First ts " << next_output_required_at << std::endl;
            }
            if (next_output_required_at < buffers.current_batch->first->back()->t){
                to_check = true;
            }

            if (buffers.current_batch->first->back()->t > (next_output_required_at - accumulation_period))
            {
                for (const Metavision::Event2d* ev : *buffers.current_batch->first) {
                    if (ev->t > (next_output_required_at - accumulation_period)) {
                        img.at<uint8_t>(ev->y, ev->x) = 255;

                        if (ev->t > next_output_required_at) {
                            next_output_required_at += period;

                            getOutput();

                            drawCurrentState();

                            cv::Mat matF;
                            ui_ptr->show_async(output);
                            Metavision::EventLoop::poll_and_dispatch();

                            resetState();
                        }

                    }
                }
            }
        }
        else {
        }
    }
    ui_ptr->set_close_flag();
    std::this_thread::sleep_for(std::chrono::microseconds(500));
    closed = true;

}
void VisualizationController::drawCurrentState() {
    auto current_state = logger->getCurrentStatus();
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    for(const std::pair<int, std::shared_ptr<OutputEntry>> current_marker: *current_state){
        for (auto tracker : current_marker.second->tracker_outputs){
            cv::circle(output, cv::Point(tracker.x, tracker.y), tracker.r, cv::Scalar(0, 0, 255), 2);
        }
        cv::Vec3d trans(current_marker.second->pose_trans.x, current_marker.second->pose_trans.y, current_marker.second->pose_trans.z);
        cv::Vec3d rot(current_marker.second->pose_rot.r_0, current_marker.second->pose_rot.r_1, current_marker.second->pose_rot.r_2);
        cv::aruco::drawAxis(output,  cam_obj->camera_matrix_cv, cam_obj->dist_coeffs, rot, trans, 0.03);
    }

}

void VisualizationController::start() {
    if(buffers.input_buffer != nullptr){
        thread_ptr = new boost::thread(&VisualizationController::runtimeLoop, this);
    }
}


void VisualizationController::getOutput() {
    cv::cvtColor(img, output, cv::COLOR_GRAY2BGR);
}

void VisualizationController::resetState(){
    img = cv::Mat::zeros(height, width, CV_8UC1);
    output = cv::Mat::zeros(height, width, CV_8UC3);
    counts = cv::Mat::zeros(height, width, CV_8UC1);
    labels = cv::Mat::zeros(height, width, CV_16UC1);
}

bool VisualizationController::isFinished() {
    return closed.load();
}




