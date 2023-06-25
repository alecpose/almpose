#include <iostream>

#include "include/utils/options.hpp"
#include "include/camera.hpp"
#include "include/utils/visualization.hpp"
#include "include/markers.hpp"
#include "include/runtime_manager.hpp"
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

void my_handler(int s){
    std::cout << "EXITING!" << std::endl;    
    exit(1); 
}

int main(int argc, char *argv[]) {

    //Parse options
    Utils::Options::Parser parser(argc, argv);

    if (parser.current_setup.cam_config.is_recording){
        std::cout << "Using recording: " << parser.current_setup.cam_config.file_path << std::endl;
    }
    else{
        std::cout << "Using camera with biases from: " << parser.current_setup.cam_config.biases_file << std::endl;
        std::cout << "Using camera config from: " << parser.current_setup.cam_config.config_file_path << std::endl;
    }

    Camera cam(&parser.current_setup.getCamConfig());
    cam.initialize_camera();

    MarkersManager markers(parser.current_setup.getMarkerConfig(), &cam);

    RuntimeManager runtime(markers, &cam, &parser.current_setup);
    runtime.buffers.setInputBuffer(cam.reader.buffers.getOutputBuffer());

    VisualizationController vis(cam.width, cam.height, &cam, &runtime);
    vis.buffers.setInputBuffer(cam.reader.buffers.getOutputBuffer());

    cam.start();

    auto now_ms = std::chrono::system_clock::now();
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::microseconds>(epoch);

    auto recording_start = std::chrono::system_clock::now();
    auto recording_start_epoch = recording_start.time_since_epoch();
    auto recording_start_value = std::chrono::duration_cast<std::chrono::microseconds>(recording_start_epoch);


    vis.start();
    runtime.start();

    cam.reader.start();

    
    bool recording = false;
    if(parser.current_setup.recording_time > 0){
        recording = true;
    }
    signal (SIGINT, my_handler);


    while (!vis.isFinished()) {
        if(recording){         
            
            auto recording_end = std::chrono::system_clock::now();
            auto recording_end_epoch = recording_end.time_since_epoch();
            auto recording_end_value = std::chrono::duration_cast<std::chrono::microseconds>(recording_end_epoch);

            if((recording_end_value.count() - recording_start_value.count()) > (parser.current_setup.recording_time * 1000000)){
                now_ms = std::chrono::system_clock::now();
                epoch = now_ms.time_since_epoch();
                value = std::chrono::duration_cast<std::chrono::microseconds>(epoch);
            }
           
            
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }

    return 0;
}
