#include "../../include/utils/buffers.hpp"

std::shared_ptr<Queue> Buffers::getOutputBuffer() {
    output_buffers_mutex->lock();
    std::shared_ptr<Queue> tmp = std::make_shared<Queue>(QUEUE_SIZE);
    output_buffers.push_back(tmp);
    output_buffers_mutex->unlock();

    return tmp;
}

bool Buffers::getBatch() {

    if (input_buffer != nullptr){
        return input_buffer->wait_dequeue_timed(current_batch, std::chrono::microseconds (10));
    }
    else{
        throw std::runtime_error("Trying to read from buffer without initializing input buffer");
    }
}

bool Buffers::getBatchAndSendFurther() {

    if(getBatch()){
        sendBatch(current_batch);
        return true;
    }
    else{
        return false;
    }
}



void Buffers::sendBatch(EventBatch to_send) {
    output_buffers_mutex->lock();
    for(auto buffer : output_buffers) {
        buffer->enqueue(to_send);
    }
    output_buffers_mutex->unlock();
}

Buffers::Buffers() {
    output_buffers_mutex = std::make_shared<std::mutex>();
    input_buffer = nullptr;
}

void Buffers::deregisterBuffer(std::shared_ptr<Queue> buffer_to_remover) {
    output_buffers_mutex->lock();
    output_buffers.erase(std::remove(output_buffers.begin(), output_buffers.end(), buffer_to_remover), output_buffers.end());
    output_buffers_mutex->unlock();
}



