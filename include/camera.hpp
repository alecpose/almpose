#ifndef FRAMEWORK_MARKERS_CAMERA_HPP
#define FRAMEWORK_MARKERS_CAMERA_HPP

#include "utils/options.hpp"
#include <metavision/sdk/driver/camera.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_hw_register.h>
#include "event_reader.hpp"
#include "utils/buffers.hpp"
class Camera {
public:
    Camera(Utils::Options::CameraSetup *setup);
    int width;
    int height;

    EventBufferReader reader;

    void initialize_camera();
    void start() {camera_object.start();};
    cv::Mat camera_matrix_cv;
    cv::Mat dist_coeffs;
    void printTiming(const Metavision::EventExtTrigger*, const Metavision::EventExtTrigger*);
    
private:
    Metavision::Camera camera_object;
    Metavision::Biases *biases;

    bool is_triggering_active = false;

    Eigen::Matrix3d camera_matrix_eigen;




};


#endif //FRAMEWORK_MARKERS_CAMERA_HPP
