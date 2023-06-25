#ifndef FRAMEWORK_MARKERS_MARKERS_HPP
#define FRAMEWORK_MARKERS_MARKERS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "utils/csvfile.h"

#include <metavision/sdk/base/events/event_cd.h>
#include "camera.hpp"
#include "utils/options.hpp"
#include <mutex>
#include <thread>
#include "utils/buffers.hpp"
#include "tracker.hpp"
#include "utils/types.hpp"
#include "logger.hpp"
class MarkersManager;

class Marker{
public:
    Marker(int id,
           std::vector<cv::Point3d> points_3d,
           std::vector<std::vector<cv::Point>> contours,
           std::vector<int> frequencies,
           long long detection_time,
           Camera *cam,
           std::shared_ptr<Logger> logger,
           MarkersManager *markers_manager
           );
    MarkersManager* markers_manager_ptr;
    std::shared_ptr<Logger> logger_ptr;
    Buffers *input_buffers_ptr= nullptr;

    std::vector<int> blob_frequencies;
    csvfile *csv;

    cv::Mat t_vec;
    cv::Mat r_vec;
//    cv::Point2f project(cv::Point3f &point);
    std::vector<cv::Point2d> projected_objectpoints;
    std::vector<cv::Point2d> detected_initial_points;
    std::vector<cv::Point3d> objectpoints_3d;
    void stopTracking();
    std::vector<std::shared_ptr<TrackedBlob>> tracked_blobs;

    cv::Point2d projected_centerpoint;
    cv::Point3d centerpoint_3d;
    int id;
    long long detected_at;

    long long final_accumulation_time = 0;
    long long accumulation_period = 100;

    bool new_batch = false;

    std::atomic_bool tracking_lost = false;

    std::atomic_bool pnp_running;
    std::atomic_bool new_result;

    std::jthread current_pnp_thread;

    std::vector<TrackerOut> blobs_for_current_pnp;
    std::vector<double> average_distances;

    Buffers buffers;
    std::shared_ptr<Deque> initial_buffer;

    std::shared_ptr<Deque> working_buffer;


    void startTracking(std::shared_ptr<Deque> deque, Buffers *input_buffers);
    void solvePnP();

private:
    double tracking_lost_threshold =  10.;
    long long current_camera_timestamp;
    long long current_pc_timestamp;
    std::shared_ptr<OutputEntry> current_output;
    cv::SolvePnPMethod solve_method = cv::SolvePnPMethod::SOLVEPNP_IPPE;
    Camera *cam_obj;
    std::shared_ptr<std::jthread> tracking_thread;
    std::vector<cv::Point> position_2d;
    void track();
    std::shared_ptr<OutputEntry> getOutput(bool detection);
};

class MarkersManager{
public:
    void deregisterMarker(Marker *marker_to_remove);
    MarkersManager(Utils::Options::MarkersSetup setup, Camera *cam);

    std::vector<std::shared_ptr<Marker>> matchMarkers(std::vector<double> frequencies,
                                     std::vector<std::vector<cv::Point>> filtered_contours,
                                     long long detection_timestamp);

    void registerDetections(std::vector<double> frequencies,
                            std::vector<std::vector<cv::Point>> filtered_contours,
                            long long detection_timestamp);

    void spawnMarkers(std::shared_ptr<Deque> deque, Buffers *input_buffers);

    void setLogger(std::shared_ptr<Logger> logger){logger_ptr = logger;};



private:
    std::shared_ptr<Logger> logger_ptr;
    std::vector<std::shared_ptr<Marker>> tracked_markers;
    std::vector<std::shared_ptr<Marker>> markers_to_spawn;
    std::vector<std::shared_ptr<Marker>> markers_to_destroy;
    std::vector<int> tracked_markers_ids;
    std::vector<bool> tracker_status;
    Camera *cam_obj;
    Utils::Options::MarkersSetup config;


    std::mutex tracked_markers_mutex;
    std::mutex tracked_markers_ids_mutex;


};


#endif //FRAMEWORK_MARKERS_MARKERS_HPP
