#include "people_detector.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <fstream>

PeopleDetector::PeopleDetector()
    : config_(Config{})
{
}

PeopleDetector::PeopleDetector(const Config& config)
    : config_(config)
{
}

void PeopleDetector::setConfig(const Config& config)
{
    config_ = config;
}

PeopleDetector::Config PeopleDetector::getConfig() const
{
    return config_;
}

std::vector<PersonDetection> PeopleDetector::detect(
    const InputCloud::ConstPtr& input,
    InputCloud::Ptr& visualization_cloud,
    Diagnostics* diagnostics) const
{
    std::ofstream trace;
    if (std::getenv("PEOPLE_PROBE_TRACE")) trace.open("probe_candidates.csv");
    Diagnostics stats;
    if (input) stats.raw_points = input->size();
    std::vector<PersonDetection> detections;

    if (!input || input->empty())
    {
        visualization_cloud.reset(new InputCloud);
        if (diagnostics) *diagnostics = stats;
        return detections;
    }

    // ============================================================
    // 1. ROI and voxel filtering
    // ============================================================

    InputCloud::Ptr filtered(new InputCloud);

    pcl::VoxelGrid<InputPoint> voxel;

    // Limit the cloud BEFORE clustering to the desired forward ROI.
    // Assumes +X forward and +Z upward; verify sensor axes.
    InputCloud::Ptr roi(new InputCloud);
    roi->reserve(input->size());
    const float pi = 3.14159265358979323846f;
    for (const auto& pt : input->points)
    {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;
        const float range = std::hypot(pt.x, pt.y);
        if (range < config_.min_range || range > config_.max_range)
            continue;
        if (config_.enable_edge_rejection &&
            std::abs(std::atan2(pt.y, pt.x) * 180.0f / pi) >
                config_.max_horizontal_angle_deg)
            continue;
        roi->push_back(pt);
    }
    stats.roi_points = roi->size();
    if (roi->empty())
    {
        visualization_cloud = roi;
        if (diagnostics) *diagnostics = stats;
        return detections;
    }
    voxel.setInputCloud(roi);

    voxel.setLeafSize(
        config_.voxel_size,
        config_.voxel_size,
        config_.voxel_size);

    voxel.filter(*filtered);

    stats.voxel_points = filtered->size();
    if (filtered->empty())
    {
        visualization_cloud = filtered;
        if (diagnostics) *diagnostics = stats;
        return detections;
    }

    // ============================================================
    // 2. RANSAC ground removal
    // ============================================================

    pcl::SACSegmentation<InputPoint> seg;

    seg.setOptimizeCoefficients(true);
    // Fit only floor-like planes among the lower returns. An unconstrained
    // dominant-plane fit selected the ceiling in the live sensor capture.
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f::UnitZ());
    seg.setEpsAngle(20.0f * 3.14159265358979323846f / 180.0f);
    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setDistanceThreshold(config_.ground_distance);
    seg.setMaxIterations(config_.ground_iterations);

    seg.setInputCloud(filtered);
    std::vector<float> heights;
    heights.reserve(filtered->size());
    for (const auto& p : filtered->points) heights.push_back(p.z);
    const auto lower = heights.begin() + heights.size() / 4;
    std::nth_element(heights.begin(), lower, heights.end());
    pcl::PointIndices::Ptr floorSeeds(new pcl::PointIndices);
    for (std::size_t i = 0; i < filtered->size(); ++i)
        if (filtered->points[i].z <= *lower) floorSeeds->indices.push_back(static_cast<int>(i));
    seg.setIndices(floorSeeds);

    pcl::PointIndices::Ptr ground(
        new pcl::PointIndices);

    pcl::ModelCoefficients::Ptr coefficients(
        new pcl::ModelCoefficients);

    seg.segment(*ground, *coefficients);
    if (trace) {
        trace << "plane";
        for (float c : coefficients->values) trace << ',' << c;
        trace << "\npoints,width,depth,height,center_x,center_y,center_z\n";
    }

    // Expand seed inliers to the complete fitted floor, preserving objects above it.
    if (!ground->indices.empty() && coefficients->values.size() == 4) {
        ground->indices.clear();
        const auto& c = coefficients->values;
        const float norm = std::sqrt(c[0]*c[0]+c[1]*c[1]+c[2]*c[2]);
        if (norm > 1e-6f) {
            for (std::size_t i=0; i<filtered->size(); ++i) {
                const auto& p=filtered->points[i];
                if (std::abs(c[0]*p.x+c[1]*p.y+c[2]*p.z+c[3])/norm <= config_.ground_distance)
                    ground->indices.push_back(static_cast<int>(i));
            }
        }
    }

    InputCloud::Ptr non_ground(new InputCloud);

    if (!ground->indices.empty())
    {
        pcl::ExtractIndices<InputPoint> extract;

        extract.setInputCloud(filtered);
        extract.setIndices(ground);
        extract.setNegative(true);

        extract.filter(*non_ground);
    }
    else
    {
        *non_ground = *filtered;
    }

    stats.non_ground_points = non_ground->size();
    stats.ground_removed_points = filtered->size() - non_ground->size();
    visualization_cloud = non_ground;

    if (non_ground->empty()) {
        if (diagnostics) *diagnostics = stats;
        return detections;
    }

    // ============================================================
    // 3. Euclidean clustering
    // ============================================================

    pcl::search::KdTree<InputPoint>::Ptr tree(
        new pcl::search::KdTree<InputPoint>);

    tree->setInputCloud(non_ground);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<InputPoint> ec;

    ec.setClusterTolerance(
        config_.cluster_tolerance);

    ec.setMinClusterSize(
        config_.min_cluster_size);

    ec.setMaxClusterSize(
        config_.max_cluster_size);

    ec.setSearchMethod(tree);
    ec.setInputCloud(non_ground);

    ec.extract(cluster_indices);
    stats.clusters = cluster_indices.size();

    // ============================================================
    // 4. Candidate cluster filtering
    // ============================================================

    for (const auto& indices : cluster_indices)
    {
        if (indices.indices.size() <
            config_.min_person_points)
        {
            ++stats.rejected_points;
            continue;
        }

        float min_x =
            std::numeric_limits<float>::max();

        float min_y =
            std::numeric_limits<float>::max();

        float min_z =
            std::numeric_limits<float>::max();

        float max_x =
            -std::numeric_limits<float>::max();

        float max_y =
            -std::numeric_limits<float>::max();

        float max_z =
            -std::numeric_limits<float>::max();

        // --------------------------------------------------------
        // Bounding box
        // --------------------------------------------------------

        for (int index : indices.indices)
        {
            const auto& p =
                non_ground->points[index];

            min_x = std::min(min_x, p.x);
            min_y = std::min(min_y, p.y);
            min_z = std::min(min_z, p.z);

            max_x = std::max(max_x, p.x);
            max_y = std::max(max_y, p.y);
            max_z = std::max(max_z, p.z);
        }

        const float width =
            max_x - min_x;

        const float depth =
            max_y - min_y;

        const float height =
            max_z - min_z;

        // --------------------------------------------------------
        // Basic human dimensions
        // --------------------------------------------------------

        if (trace) trace << indices.indices.size() << ',' << width << ',' << depth << ',' << height
            << ',' << (min_x+max_x)*0.5f << ',' << (min_y+max_y)*0.5f << ',' << (min_z+max_z)*0.5f << '\n';

        if (height < config_.min_height ||
            height > config_.max_height)
        {
            ++stats.rejected_height;
            continue;
        }

        if (width < config_.min_width ||
            width > config_.max_width)
        {
            ++stats.rejected_width;
            continue;
        }

        if (depth < config_.min_depth ||
            depth > config_.max_depth)
        {
            ++stats.rejected_depth;
            continue;
        }

        // --------------------------------------------------------
        // Cluster center
        // --------------------------------------------------------

        const Eigen::Vector3f center(
            0.5f * (min_x + max_x),
            0.5f * (min_y + max_y),
            0.5f * (min_z + max_z));

        // ========================================================
        // 5. Range filtering
        // ========================================================

        const float horizontal_range =
            std::sqrt(
                center.x() * center.x() +
                center.y() * center.y());

        if (horizontal_range <
            config_.min_range)
        {
            ++stats.rejected_range;
            continue;
        }

        if (horizontal_range >
            config_.max_range)
        {
            ++stats.rejected_range;
            continue;
        }

        // ========================================================
        // 6. Edge / FOV rejection
        // ========================================================

        if (config_.enable_edge_rejection)
        {
            const float angle =
                std::atan2(
                    center.y(),
                    center.x());

            const float angleDeg =
                std::abs(
                    angle * 180.0f /
                    static_cast<float>(M_PI));

            if (angleDeg >
                config_.max_horizontal_angle_deg)
            {
                ++stats.rejected_angle;
                continue;
            }
        }

        // ========================================================
        // 7. Flat-object rejection
        // ========================================================
        //
        // Estimate the 3D covariance of the cluster.
        //
        // A door/wall tends to have one very small eigenvalue
        // because most points lie close to a plane.
        //
        // A person normally occupies more 3D volume.
        //
        // flatness = smallest eigenvalue /
        //            sum of eigenvalues
        //
        // Very small value -> strongly planar.
        //
        // ========================================================

        if (config_.enable_flat_object_rejection &&
            indices.indices.size() >= 10)
        {
            Eigen::Vector3f mean =
                Eigen::Vector3f::Zero();

            for (int index : indices.indices)
            {
                const auto& p =
                    non_ground->points[index];

                mean.x() += p.x;
                mean.y() += p.y;
                mean.z() += p.z;
            }

            mean /=
                static_cast<float>(
                    indices.indices.size());

            Eigen::Matrix3f covariance =
                Eigen::Matrix3f::Zero();

            for (int index : indices.indices)
            {
                const auto& p =
                    non_ground->points[index];

                Eigen::Vector3f d(
                    p.x - mean.x(),
                    p.y - mean.y(),
                    p.z - mean.z());

                covariance +=
                    d * d.transpose();
            }

            covariance /=
                static_cast<float>(
                    indices.indices.size());

            Eigen::SelfAdjointEigenSolver<
                Eigen::Matrix3f> solver(
                    covariance);

            if (solver.info() ==
                Eigen::Success)
            {
                const auto eigenvalues =
                    solver.eigenvalues();

                const float sum =
                    eigenvalues[0] +
                    eigenvalues[1] +
                    eigenvalues[2];

                if (sum > 1e-6f)
                {
                    const float flatness =
                        eigenvalues[0] / sum;

                    /*
                     * Important:
                     *
                     * A very small flatness means
                     * strongly planar.
                     *
                     * Therefore reject when flatness
                     * is TOO SMALL.
                     */
                    const float minimumPlanarity =
                        1.0f -
                        config_.max_flatness;

                    if (flatness <
                        minimumPlanarity)
                    {
                        ++stats.rejected_flatness;
                        continue;
                    }
                }
            }
        }

        // ========================================================
        // 8. Accepted person
        // ========================================================

        PersonDetection detection;

        detection.center = center;

        detection.width = width;
        detection.depth = depth;
        detection.height = height;

        detection.points =
            indices.indices.size();

        detections.push_back(
            detection);
        ++stats.accepted;
    }

    if (diagnostics) *diagnostics = stats;
    return detections;
}
