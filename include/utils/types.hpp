#ifndef FRAMEWORK_MARKERS_TYPES_HPP
#define FRAMEWORK_MARKERS_TYPES_HPP

#include <metavision/sdk/base/events/event2d.h>
#include <readerwriterqueue/readerwriterqueue.h>
#include <memory>
#include <stack>
#include <deque>
#include "../external/concurrentqueue.h"

struct RotationOut {
    double r_0 = 0.;
    double r_1 = 0.;
    double r_2 = 0.;
};

struct TranslationOut {
    double x = 0.;
    double y = 0.;
    double z = 0.;
};

struct TrackerOut {
    double x = 0.;
    double y = 0.;
    double r = 0.;
    int freq = 0.;
};

struct OutputEntry {
    long long camera_timestamp = 0;
    long long pc_timestamp = 0;
    unsigned int id = 0;
    bool detection = false;
    std::vector<TrackerOut> tracker_outputs;
    TranslationOut pose_trans;
    RotationOut pose_rot;
};


typedef moodycamel::ConcurrentQueue<std::shared_ptr<OutputEntry>> OutputQueue;

typedef std::shared_ptr<std::pair<std::shared_ptr<std::vector<const Metavision::Event2d*>>,
                        std::shared_ptr<std::vector<Metavision::Event2d>>>> EventBatch;

typedef std::pair<std::shared_ptr<std::vector<const Metavision::Event2d*>>,
        std::shared_ptr<std::vector<Metavision::Event2d>>> EventBatchPair;
typedef moodycamel::BlockingReaderWriterQueue<EventBatch> Queue;

typedef std::stack<EventBatch> Stack;
typedef std::deque<EventBatch> Deque;

#endif //FRAMEWORK_MARKERS_TYPES_HPP
