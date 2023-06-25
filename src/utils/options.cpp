#include <iostream>
#include "../../include/utils/options.hpp"

Utils::Options::Parser::Parser(int argc, char **argv){

    std::string camera_config_filepath;
    std::string recording_filepath;
    std::string biases_filepath;
    std::string markers_config_filepath;
    std::string ros_parent_name;

    int recording_time;

    po::options_description general_opt("General Options");
    general_opt.add_options()
            ("help,h", "Help screen")
            ("input_file,i", po::value<std::string>(&recording_filepath), "Recording file (raw)")
            ("camera_config_file,c", po::value<std::string>(&camera_config_filepath)->required(), "config file (yaml)")
            ("biases_file,b", po::value<std::string>(&biases_filepath), "Biases file (bias)")
            ("markers_config_filepath,m", po::value<std::string>(&markers_config_filepath)->required(), "Description file for markers")
            ("ros_parent,r", po::value<std::string>(&ros_parent_name), "Name of the parent TF")
            ("csv_enable,csv", po::bool_switch()->default_value(false), "Enables csv logging")
            ("synchro,sync", po::bool_switch()->default_value(false), "Enables synchronisation")
            ("recording_time,t", po::value<int>(&recording_time), "Synchronisation time (ms)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, general_opt), vm);
    po::notify(vm);

    if (vm.count("help"))
        std::cout << general_opt << '\n';


    //Camera config
    CameraSetup cam_config;
    if (vm.count("ros_parent")){
        current_setup.parent_tf_name = vm["ros_parent"].as<std::string>();
        std::cout << "ROS TF enabled - parent TF name: " << current_setup.parent_tf_name << std::endl;
    }
    else{
        current_setup.parent_tf_name = "";
    }
    
    if (vm.count("recording_time")){
        current_setup.recording_time = vm["recording_time"].as<int>();
    }

    current_setup.synchro = vm["synchro"].as<bool>();
    if(current_setup.synchro){
        std::cout << "Synchronization enabled" << std::endl;
    }

    current_setup.csv_logging_enabled = vm["csv_enable"].as<bool>();
    if(current_setup.csv_logging_enabled){
        std::cout << "CSV logging enabled!" << std::endl;
    }


    if (vm.count("input_file")){
        cam_config.is_recording = true;
        cam_config.file_path = vm["input_file"].as<std::string>();
    }
    else {
        if (!vm.count("biases_file")){
            throw po::invalid_option_value("No biases file included");
        }
        else{
            cam_config.biases_file = vm["biases_file"].as<std::string>();
        }
    }
    cam_config.config_file_path = vm["camera_config_file"].as<std::string>();

    YAML::Node config = YAML::LoadFile(vm["camera_config_file"].as<std::string>());

    std::vector<double> camera_matrix_vect = config["CameraMatrix"]["data"].as<std::vector<double>>();
    std::vector<double> dist_coeffs_vect = config["DistCoeff"]["data"].as<std::vector<double>>();
    int dist_coeffs_len = config["DistCoeff"]["len"].as<int>();

    assert((int)camera_matrix_vect.size() == 9);
    assert((int)dist_coeffs_vect.size() == dist_coeffs_len);

    //camera matrices
    cv::Mat camera_matrix = cv::Mat1d(3, 3);
    cam_config.camera_matrix_cv = camera_matrix;
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            cam_config.camera_matrix_cv.at<double>(row, col) = camera_matrix_vect[row * 3 + col];
            cam_config.camera_matrix_eigen(row, col) = camera_matrix_vect[row * 3 + col];
        }
    }
    //dist coeffs
    cv::Mat dist_coeffs = cv::Mat1d(1,5);
    cam_config.dist_coeffs = dist_coeffs;

    for(std::vector<int>::size_type i = 0; i != dist_coeffs_vect.size(); i++){
        cam_config.dist_coeffs.at<double>(0, i) = dist_coeffs_vect[i];
    }

    if (config["ExternalTriggers"]){
        cam_config.is_using_triggers = true;
        cam_config.triggers_channel = config["ExternalTriggers"]["channel_id"].as<int>();
    }

    current_setup.setCamConfig(cam_config);

    //Marker configuration
    MarkersSetup marker_config;

    YAML::Node markers_config = YAML::LoadFile(vm["markers_config_filepath"].as<std::string>());
    for (size_t i = 0; i < markers_config["Markers"].size(); i++){
        auto current_marker = markers_config["Markers"][i];
        int id = current_marker["ID"].as<int>();
        std::vector<cv::Point3d> markers_points;
        std::vector<double> markers_frequencies;
        for (size_t j = 0; j < current_marker["Points"].size(); j++){
            auto current_point = current_marker["Points"][j];
            markers_points.emplace_back(current_point['x'].as<double>(),
                                         current_point['y'].as<double>(),
                                         current_point['z'].as<double>());
            markers_frequencies.emplace_back(current_point["freq"].as<int>());

        }


        marker_config.coordinates.push_back(markers_points);
        marker_config.frequencies.push_back(markers_frequencies);
        marker_config.ids.push_back(id);
    }
    current_setup.setMarkerConfig(marker_config);

}

Utils::Options::CameraSetup &Utils::Options::Setup::getCamConfig() {
    return cam_config;
}

void Utils::Options::Setup::setCamConfig(Utils::Options::CameraSetup &camConfig) {
    cam_config = camConfig;
}

Utils::Options::MarkersSetup &Utils::Options::Setup::getMarkerConfig() {
    return marker_config;
}

void Utils::Options::Setup::setMarkerConfig(Utils::Options::MarkersSetup &markerConfig) {
    marker_config = markerConfig;
}
