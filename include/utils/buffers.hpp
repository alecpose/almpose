#ifndef FRAMEWORK_MARKERS_BUFFERS_HPP
#define FRAMEWORK_MARKERS_BUFFERS_HPP

#include "types.hpp"
#include <chrono>
#include <mutex>
#define QUEUE_SIZE 1024

class Buffers {
public:
    Buffers();
    std::shared_ptr<Queue> getOutputBuffer();
    std::shared_ptr<Queue> getInputBuffer(){return input_buffer;};
    void deregisterBuffer(std::shared_ptr<Queue> buffer_to_remover);
    void setInputBuffer(std::shared_ptr<Queue> input){input_buffer = input;};
    bool getBatch();
    bool getBatchAndSendFurther();

    void sendBatch(EventBatch batch);

    EventBatch current_batch;

    std::shared_ptr<std::mutex> output_buffers_mutex;

    std::shared_ptr<Queue> input_buffer;
    std::vector<std::shared_ptr<Queue>> output_buffers;
};


#endif //FRAMEWORK_MARKERS_BUFFERS_HPP
