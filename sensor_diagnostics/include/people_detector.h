#pragma once

#include "people_types.h"
#include <vector>
#include <cstddef>

class PeopleDetector
{
public:
    struct Config
    {
        // ------------------------------------------------------------
        // Point-cloud preprocessing
        // ------------------------------------------------------------

        float voxel_size = 0.03f;

        float ground_distance = 0.12f;
        int ground_iterations = 100;


        // ------------------------------------------------------------
        // Clustering
        // ------------------------------------------------------------

        float cluster_tolerance = 0.20f;

        int min_cluster_size = 30;
        int max_cluster_size = 5000;


        // ------------------------------------------------------------
        // Human dimensions
        // ------------------------------------------------------------

        float min_height = 0.60f;
        float max_height = 2.20f;

        float min_width = 0.15f;
        float max_width = 1.00f;

        float min_depth = 0.15f;
        float max_depth = 0.80f;

        std::size_t min_person_points = 45;


        // ------------------------------------------------------------
        // Detection range
        // ------------------------------------------------------------

        // Horizontal distance from the LiDAR.
        float min_range = 0.0f;
        float max_range = 100.0f;


        // ------------------------------------------------------------
        // Edge / FOV rejection
        // ------------------------------------------------------------

        bool enable_edge_rejection = false;

        // Maximum accepted horizontal angle from the forward
        // direction of the LiDAR.
        //
        // Example:
        //   45 deg = accept objects within ±45 degrees
        //   60 deg = accept objects within ±60 degrees
        //
        // Smaller value = stronger rejection near the sides.
        float max_horizontal_angle_deg = 45.0f;


        // ------------------------------------------------------------
        // Flat-object rejection
        // ------------------------------------------------------------

        bool enable_flat_object_rejection = false;
        float max_flatness = 0.90f;
    };


    struct Diagnostics {
        std::size_t raw_points = 0, roi_points = 0, voxel_points = 0;
        std::size_t ground_removed_points = 0, non_ground_points = 0;
        std::size_t clusters = 0, rejected_points = 0, rejected_height = 0;
        std::size_t rejected_width = 0, rejected_depth = 0;
        std::size_t rejected_range = 0, rejected_angle = 0;
        std::size_t rejected_flatness = 0, accepted = 0;
    };

    PeopleDetector();
    explicit PeopleDetector(const Config& config);


    // ------------------------------------------------------------
    // Runtime configuration
    // ------------------------------------------------------------

    void setConfig(const Config& config);

    Config getConfig() const;


    // ------------------------------------------------------------
    // Detection
    // ------------------------------------------------------------

    std::vector<PersonDetection> detect(
        const InputCloud::ConstPtr& input,
        InputCloud::Ptr& visualization_cloud,
        Diagnostics* diagnostics = nullptr) const;


private:

    Config config_;
};
