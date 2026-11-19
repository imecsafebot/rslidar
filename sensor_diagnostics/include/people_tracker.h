#pragma once

#include "people_types.h"
#include <vector>

class PeopleTracker
{
public:
    struct Config
    {
        float max_association_distance = 1.0f;
        float max_prediction_distance = 2.0f;
        int max_missed_frames = 25;
        float min_movement = 0.30f;
        float fallback_dt = 0.10f;
    };

    PeopleTracker();
    explicit PeopleTracker(const Config& config);

    void setConfig(const Config& config);
    Config getConfig() const;

    void update(
        std::vector<PersonDetection>& detections,
        double timestamp_seconds);

    // Useful when the user changes tracking parameters
    // and wants to start tracking fresh.
    void reset();

private:
    struct Track
    {
        int id = -1;

        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        Eigen::Vector3f start_center = Eigen::Vector3f::Zero();
        Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

        float width = 0.0f;
        float depth = 0.0f;
        float height = 0.0f;
        std::size_t points = 0;

        double last_timestamp = 0.0;
        int missed_frames = 0;
        int observations = 0;

        float speed = 0.0f;
        bool moving = false;
    };

    Config config_;

    int next_id_ = 0;
    std::vector<Track> active_;

    static float distance2D(
        const Eigen::Vector3f& a,
        const Eigen::Vector3f& b);

    void removeLostTracks();
};
