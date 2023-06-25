#ifndef FRAMEWORK_MARKERS_TRACKER_HPP
#define FRAMEWORK_MARKERS_TRACKER_HPP
#define ROI_SIZE 3

#include <eigen3/Eigen/Dense>
#include <metavision/sdk/base/events/event_cd.h>
#include <opencv2/opencv.hpp>
#include "utils/types.hpp"

class TrackedBlob {
public:
    TrackedBlob(cv::Point2d &center, int freq, long long detected_at, double radius = 5.);
    cv::Point2d center_position;
    int frequency;

    std::deque<const Metavision::Event2d*> event_buffer;

    inline bool is_event_inside(const Metavision::Event2d *ev) const {
        return cv::norm(cv::Point2d(ev->x, ev->y) - center_position) < radius;
    }

    inline double calculate_distance(const Metavision::Event2d *ev) const {
        return cv::norm(cv::Point2d(ev->x, ev->y) - center_position);
    }

//    bool is_point_inside(Eigen::Vector2f &position) {
//        return (position - center_position).norm() < radius;
//    }

    void cluster_analytics();
    int getUpdateCount() const{return update_count_since_pnp;};
    void resetUpdateCount() {update_count_since_pnp = 0;};
    int update_count_since_pnp = 0;

    TrackerOut get_output();

    bool update(const Metavision::Event2d *ev);
    double radius;
    double current_distance;
    double distance_sum = 0;

    bool first = true;
    int update_count = 0;

    double alpha = 0.9;       // update coeff of cluster x,y parameter
    double delta = 0.1;       // update coeff of cluster N parameter
    double beta = 0.020;          // update coeff of ROI
    double gamma = 0.01;       // update coeff of ROI radius

    const double max_roi_radius = 16.;

    short unsigned int x_min;
    short unsigned int x_max;
    short unsigned int y_min;
    short unsigned int y_max;

    float period = 0;

    long long last_update = 0;

    unsigned int acc_x = 0;
    unsigned int acc_y = 0;
    long long acc_time = 0;

    double x_center = 0;
    double y_center = 0;
    long long t_cluster = 0;
    double N_cluster = 1.;

    short unsigned int size_x = 0;
    short unsigned int size_y = 0;

    float avg_distance= 0;

};



#endif //FRAMEWORK_MARKERS_TRACKER_HPP
