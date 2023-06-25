#include "../include/event_reader.hpp"

EventBufferReader::EventBufferReader(){
    start_accumulation = false;
    synchronized = false;
}

void EventBufferReader::readEvents(const Metavision::Event2d *ev_begin, const Metavision::Event2d *ev_end) {
    if(ev_begin != nullptr && ev_end != nullptr){
        if(!start_accumulation.load() || !synchronized){
            if (!synchronized){
                auto now_ms = std::chrono::system_clock::now();
                auto epoch = now_ms.time_since_epoch();
                auto value = std::chrono::duration_cast<std::chrono::microseconds>(epoch);
                std::cout << "Synchro " << ev_begin->t << " at " << value.count() << std::endl;
                synchronized = true;
            }
        }
        else{
            input_evt_vector = std::make_shared<std::vector<Metavision::Event2d>>();
            input_pointers_vector = std::make_shared<std::vector<const Metavision::Event2d*>>();


            std::transform(ev_begin, ev_end,
                           std::back_inserter(*input_evt_vector), [](Metavision::Event2d e) { return e; });
            std::transform(input_evt_vector->begin(), input_evt_vector->end(),
                           std::back_inserter(*input_pointers_vector), [](const Metavision::Event2d& e) { return &e; });

            EventBatch output_pair = std::make_shared<EventBatchPair>(input_pointers_vector, input_evt_vector);
            buffers.sendBatch(output_pair);
        }
    }


}
