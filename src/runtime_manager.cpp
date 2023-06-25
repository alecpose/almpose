#include "../include/runtime_manager.hpp"

RuntimeManager::RuntimeManager(MarkersManager &markers_manager, Camera *cam, Utils::Options::Setup *setup) {
    logger = std::make_shared<Logger>(setup->csv_logging_enabled, setup->parent_tf_name);
    markers = &markers_manager;
    markers->setLogger(logger);
    detector = new DetectionAlgorithm(cam->width, cam->height, markers, logger);
}

void RuntimeManager::start() {
    logger->start();

    detector->buffers.setInputBuffer(buffers.getInputBuffer());
    detector->start();


}




