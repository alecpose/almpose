#include "../include/logger.hpp"


Logger::Logger(bool csv_logging, std::string ros_tf_parent_name) {
    output_queue = std::make_shared<OutputQueue>(20000);
    current_outputs_map = std::make_shared<std::unordered_map<int, std::shared_ptr<OutputEntry>>>();
    output_queue_ptr = std::make_shared<OutputQueue>();
    
    csv_logging_enabled = csv_logging;
    ros_tf_parent = ros_tf_parent_name;


    record_status = false;

    if (csv_logging_enabled) {
        char buffer [80];
        time_t t = time(0);
        struct tm * now = localtime( & t );
        strftime (buffer,80,"%Y-%m-%d-%H-%M-%S.csv",now);
        csv = new csvfile(buffer);
        *csv << "PC_TS" << "C_TS" << "ID" << "X" << "Y" << "Z" << "R1" << "R2" << "R3" << "w" << "DET" << endrow;
    }
}

void Logger::start() {
    thread_ptr = std::make_shared<std::jthread>(&Logger::loggerLoop, this);
}

[[noreturn]] void Logger::loggerLoop() {
    while (true) {
        if(output_queue->try_dequeue(current_object)){


            if(csv_logging_enabled){
                logToCsv(current_object);
            }

            if(record_status.load()){

                (*current_outputs_map)[current_object->id] = current_object;

                logged_outputs++;
                if(logged_outputs > 5){
                    record_status.store(false);
                    logged_outputs = 0;
                }
            }
            
        }
        else{
            std::this_thread::sleep_for(std::chrono::microseconds(5));
        }
    }
}

std::shared_ptr<std::unordered_map<int, std::shared_ptr<OutputEntry>>> Logger::getCurrentStatus() {
    current_outputs_map = std::make_shared<std::unordered_map<int, std::shared_ptr<OutputEntry>>>();
    record_status.store(true);
    std::chrono::time_point start = std::chrono::steady_clock::now();
    while(record_status.load() && std::chrono::steady_clock::now() - start < std::chrono::microseconds (10000  )){
        std::this_thread::sleep_for(std::chrono::microseconds(5));
    }
    if(record_status.load()){
        return std::make_shared<std::unordered_map<int, std::shared_ptr<OutputEntry>>>();
    }
    else{
        return current_outputs_map;
    }

}


void Logger::logToCsv(std::shared_ptr<OutputEntry> object) {

    cv::Mat rvec(3, 1, CV_64FC1);
            rvec.at<double>(0) = object->pose_rot.r_0;
            rvec.at<double>(1) = object->pose_rot.r_1;
            rvec.at<double>(2) = object->pose_rot.r_2;
        
            double angle = cv::norm(rvec);
            cv::normalize(rvec, rvec);


    *csv << object->pc_timestamp
        << object->camera_timestamp
        << object->id
        << object->pose_trans.x
        << object->pose_trans.y
        << object->pose_trans.z
        << rvec.at<double>(0) * sin(angle/2.0)
        << rvec.at<double>(1) * sin(angle/2.0)
        << rvec.at<double>(2) * sin(angle/2.0)
        << cos(angle/2.0)
        << object->detection
        << endrow;
    // std::cout << object->pose_trans.x << " " << object->pose_trans.y << " " << object->pose_trans.z << std::endl;
}







