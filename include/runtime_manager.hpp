#ifndef FRAMEWORK_MARKERS_RUNTIME_MANAGER_HPP
#define FRAMEWORK_MARKERS_RUNTIME_MANAGER_HPP

#include "utils/buffers.hpp"
#include "utils/types.hpp"
#include "markers.hpp"
#include "detection_algorithm.hpp"
#include "camera.hpp"
#include <thread>
#include "logger.hpp"
#include "utils/options.hpp"

class RuntimeManager {
public:
    RuntimeManager(MarkersManager &markers_manager, Camera *cam, Utils::Options::Setup *setup);
    Buffers buffers;
    MarkersManager* markers;

    DetectionAlgorithm* detector;

    std::shared_ptr<Logger> logger;

    void start();

    std::shared_ptr<Logger> getLogger(){return logger;};

private:
    [[noreturn]] void runtimeLoop();
    std::thread tracker_mapper;
};


#endif //FRAMEWORK_MARKERS_RUNTIME_MANAGER_HPP
