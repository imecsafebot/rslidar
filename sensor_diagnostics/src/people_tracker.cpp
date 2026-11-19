#include "people_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>

PeopleTracker::PeopleTracker()
    : config_(Config{})
{
}

PeopleTracker::PeopleTracker(const Config& config)
    : config_(config)
{
}

void PeopleTracker::setConfig(const Config& config)
{
    config_ = config;
}

PeopleTracker::Config PeopleTracker::getConfig() const
{
    return config_;
}

void PeopleTracker::reset()
{
    active_.clear();
    next_id_ = 0;
}

float PeopleTracker::distance2D(
    const Eigen::Vector3f& a,
    const Eigen::Vector3f& b)
{
    const float dx = a.x() - b.x();
    const float dy = a.y() - b.y();

    return std::sqrt(dx * dx + dy * dy);
}

void PeopleTracker::update(
    std::vector<PersonDetection>& detections,
    double timestamp_seconds)
{
    std::vector<bool> used(detections.size(), false);

    // ------------------------------------------------------------
    // Associate existing tracks with new detections
    // ------------------------------------------------------------
    for (auto& track : active_)
    {
        const double raw_dt =
            timestamp_seconds - track.last_timestamp;

        const float dt =
            (raw_dt > 0.001 && raw_dt < 10.0)
                ? static_cast<float>(raw_dt)
                : config_.fallback_dt;

        const Eigen::Vector3f predicted =
            track.center + track.velocity * dt;

        float allowed =
            config_.max_association_distance;

        if (track.missed_frames > 0)
        {
            allowed +=
                0.25f *
                static_cast<float>(track.missed_frames);
        }

        allowed = std::min(
            allowed,
            config_.max_prediction_distance);

        int best_index = -1;

        float best_distance =
            std::numeric_limits<float>::max();

        for (std::size_t i = 0;
             i < detections.size();
             ++i)
        {
            if (used[i])
                continue;

            const float d =
                distance2D(
                    predicted,
                    detections[i].center);

            if (d <= allowed &&
                d < best_distance)
            {
                best_distance = d;
                best_index =
                    static_cast<int>(i);
            }
        }

        // No matching detection.
        if (best_index < 0)
        {
            ++track.missed_frames;
            continue;
        }

        used[best_index] = true;

        auto& detection =
            detections[best_index];

        const Eigen::Vector3f previous =
            track.center;

        const Eigen::Vector3f displacement =
            detection.center - previous;

        const float measured_speed =
            displacement.head<2>().norm() / dt;

        // Velocity smoothing.
        constexpr float alpha = 0.5f;

        track.velocity =
            alpha * (displacement / dt) +
            (1.0f - alpha) * track.velocity;

        track.center = detection.center;

        track.width =
            detection.width;

        track.depth =
            detection.depth;

        track.height =
            detection.height;

        track.points =
            detection.points;

        track.speed =
            alpha * measured_speed +
            (1.0f - alpha) * track.speed;

        track.last_timestamp =
            timestamp_seconds;

        track.missed_frames = 0;

        ++track.observations;

        const float total_displacement =
            distance2D(
                track.center,
                track.start_center);

        track.moving =
            total_displacement >=
            config_.min_movement;

        detection.id =
            track.id;

        detection.speed =
            track.speed;

        detection.moving =
            track.moving;
    }

    // ------------------------------------------------------------
    // Create tracks for unmatched detections
    // ------------------------------------------------------------
    for (std::size_t i = 0;
         i < detections.size();
         ++i)
    {
        if (used[i])
            continue;

        const auto& detection =
            detections[i];

        Track track;

        track.id =
            next_id_++;

        track.center =
            detection.center;

        track.start_center =
            detection.center;

        track.velocity =
            Eigen::Vector3f::Zero();

        track.width =
            detection.width;

        track.depth =
            detection.depth;

        track.height =
            detection.height;

        track.points =
            detection.points;

        track.last_timestamp =
            timestamp_seconds;

        track.observations = 1;
        track.speed = 0.0f;
        track.moving = false;

        active_.push_back(track);

        detections[i].id =
            track.id;

        detections[i].speed =
            0.0f;

        detections[i].moving =
            false;
    }

    removeLostTracks();
}

void PeopleTracker::removeLostTracks()
{
    active_.erase(
        std::remove_if(
            active_.begin(),
            active_.end(),
            [this](const Track& track)
            {
                return
                    track.missed_frames >
                    config_.max_missed_frames;
            }),
        active_.end());
}
