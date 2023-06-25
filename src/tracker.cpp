#include "../include/tracker.hpp"

TrackedBlob::TrackedBlob(cv::Point2d &center, int freq,long long detected_at, double radius) {
    frequency = freq;
    last_update = detected_at;
    period = float(1000000/frequency);
    center_position = cv::Point2d(center.x, center.y);
    x_center = center.x;
    y_center = center.y;
    this->radius = radius;
    avg_distance = radius;
}

bool TrackedBlob::update(const Metavision::Event2d *ev) {
    current_distance = calculate_distance(ev);
    if (current_distance < radius){
        if (update_count == 200) {

            cluster_analytics();
            update_count = 0;

            first = false;
            update_count = 0;
            acc_x = 0;
            acc_y = 0;
            acc_time = 0;



            distance_sum = 0;

            x_min = ev->x;
            x_max = ev->x;
            y_min = ev->y;
            y_max = ev->y;
        }
        else{
            if (ev->x < x_min) x_min = ev->x;
            if (ev->x > x_max) x_max = ev->x;
            if (ev->y < y_min) y_min = ev->y;
            if (ev->y > y_max) y_max = ev->y;
        }
        distance_sum += current_distance;
        last_update = ev->t;
        center_position.x = (1 - beta) * center_position.x + beta * ev->x;
        center_position.y = (1. - beta) * center_position.y + beta * ev->y;
        acc_time += ev->t;
        acc_x += ev->x;
        acc_y += ev->y;
        update_count++;
        update_count_since_pnp++;

        return true;
    }
    return false;
}

void TrackedBlob::cluster_analytics() {
    first = true;
    if(update_count < 2) return;

    size_x = x_max - x_min;
    size_y = y_max - y_min;
    avg_distance = distance_sum / update_count;

    radius = std::min(max_roi_radius, ((1. - gamma) * radius + (2.3 * gamma * avg_distance)));
}

TrackerOut TrackedBlob::get_output() {
    return TrackerOut{.x = center_position.x, .y = center_position.y, .r = radius, .freq=frequency};
}
