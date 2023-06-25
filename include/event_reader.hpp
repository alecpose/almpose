#ifndef FRAMEWORK_MARKERS_EVENT_READER_HPP
#define FRAMEWORK_MARKERS_EVENT_READER_HPP

#include <metavision/sdk/base/events/event2d.h>
#include <memory>
#include "utils/types.hpp"
#include "utils/buffers.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
class EventBufferReader {
public:
    EventBufferReader();

    Buffers buffers;
    void start(){start_accumulation=true;};
    //test
    void readEvents(const Metavision::Event2d *ev_begin, const Metavision::Event2d *ev_end);
    std::pair<long long, long long> getFirstTimestamps(){return std::make_pair(pc_first_timestamp.load(), camera_first_timestamp.load());};
private:
    std::shared_ptr<std::vector<Metavision::Event2d>> input_evt_vector;
    std::shared_ptr<std::vector<const Metavision::Event2d*>> input_pointers_vector;
    std::atomic<bool> start_accumulation;
    std::atomic<long long> pc_first_timestamp;
    std::atomic<long long> camera_first_timestamp;
    bool synchronized;
};



#endif //FRAMEWORK_MARKERS_EVENT_READER_HPP
