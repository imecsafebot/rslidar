#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/common.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <string>

namespace fs = std::filesystem;

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;


// ============================================================
// Detection
// ============================================================

struct Detection
{
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    Eigen::Vector4f center;

    float width;
    float depth;
    float height;

    std::size_t points;
};


// ============================================================
// Track
// ============================================================

struct Track
{
    int id;

    Eigen::Vector4f center;
    Eigen::Vector4f start_center;

    float width;
    float depth;
    float height;

    std::size_t points;

    int last_frame;
    int missed_frames;

    float speed;
    bool moving;

    Eigen::Vector4f velocity;

    int observations;
};


// ============================================================
// Configuration
// ============================================================

// Detection parameters

constexpr float VOXEL_SIZE = 0.05f;

constexpr float GROUND_DISTANCE = 0.12f;

constexpr float CLUSTER_TOLERANCE = 0.25f;

constexpr int MIN_CLUSTER_SIZE = 30;
constexpr int MAX_CLUSTER_SIZE = 5000;


// Human geometry

constexpr float MIN_HEIGHT = 1.30f;
constexpr float MAX_HEIGHT = 2.20f;

constexpr float MIN_WIDTH = 0.25f;
constexpr float MAX_WIDTH = 1.00f;

constexpr float MIN_DEPTH = 0.20f;
constexpr float MAX_DEPTH = 0.90f;

constexpr int MIN_PERSON_POINTS = 50;


// Tracking

// Maximum distance between a detection and an existing track.
// Increase this if people move a lot between frames.
constexpr float MAX_ASSOCIATION_DISTANCE = 1.0f;

// A track is kept alive this many frames without detection.
constexpr int MAX_MISSED_FRAMES = 25;


// Object must move at least this much from its original
// position before we consider it moving.
constexpr float MIN_MOVEMENT = 0.30f;


// Frame interval.
//
// If your PCDs are consecutive sensor frames and you know
// the actual frame rate, change this value.
//
// Example:
// 10 Hz -> 0.1
constexpr float FRAME_TIME = 0.1f;


// ============================================================
// Numeric frame number
// ============================================================

int getFrameNumber(const fs::path& p)
{
    std::string name = p.stem().string();

    std::size_t pos = name.find_last_of('_');

    if (pos == std::string::npos)
        return -1;

    try
    {
        return std::stoi(name.substr(pos + 1));
    }
    catch (...)
    {
        return -1;
    }
}


// ============================================================
// PCD sorting
// ============================================================

bool frameNumberLess(
    const fs::path& a,
    const fs::path& b)
{
    return getFrameNumber(a) <
           getFrameNumber(b);
}


// ============================================================
// Find PCD files
// ============================================================

bool isPCD(const fs::path& p)
{
    if (!p.has_extension())
        return false;

    std::string ext =
        p.extension().string();

    std::transform(
        ext.begin(),
        ext.end(),
        ext.begin(),
        [](unsigned char c)
        {
            return std::tolower(c);
        });

    return ext == ".pcd";
}


std::vector<fs::path> findPCDFiles(
    const std::string& directory)
{
    std::vector<fs::path> files;

    for (const auto& entry :
         fs::directory_iterator(directory))
    {
        if (entry.is_regular_file() &&
            isPCD(entry.path()))
        {
            files.push_back(entry.path());
        }
    }

    std::sort(
        files.begin(),
        files.end(),
        frameNumberLess);

    return files;
}


// ============================================================
// Load PCD
// ============================================================

CloudT::Ptr loadCloud(
    const std::string& filename)
{
    CloudT::Ptr cloud(
        new CloudT);

    if (pcl::io::loadPCDFile<PointT>(
            filename,
            *cloud) != 0)
    {
        std::cerr
            << "Could not read: "
            << filename
            << std::endl;

        return nullptr;
    }

    return cloud;
}


// ============================================================
// Voxel filter
// ============================================================

CloudT::Ptr voxelFilter(
    const CloudT::Ptr& input)
{
    CloudT::Ptr output(
        new CloudT);

    pcl::VoxelGrid<PointT> voxel;

    voxel.setInputCloud(input);

    voxel.setLeafSize(
        VOXEL_SIZE,
        VOXEL_SIZE,
        VOXEL_SIZE);

    voxel.filter(*output);

    return output;
}


// ============================================================
// Remove ground
// ============================================================

CloudT::Ptr removeGround(
    const CloudT::Ptr& input)
{
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);

    seg.setModelType(
        pcl::SACMODEL_PLANE);

    seg.setMethodType(
        pcl::SAC_RANSAC);

    seg.setDistanceThreshold(
        GROUND_DISTANCE);

    seg.setMaxIterations(300);

    seg.setInputCloud(input);

    pcl::PointIndices::Ptr ground(
        new pcl::PointIndices);

    pcl::ModelCoefficients::Ptr coefficients(
        new pcl::ModelCoefficients);

    seg.segment(
        *ground,
        *coefficients);

    if (ground->indices.empty())
    {
        std::cerr
            << "Warning: ground plane was not found."
            << std::endl;

        return input;
    }

    pcl::ExtractIndices<PointT> extract;

    CloudT::Ptr objects(
        new CloudT);

    extract.setInputCloud(input);

    extract.setIndices(ground);

    extract.setNegative(true);

    extract.filter(*objects);

    return objects;
}


// ============================================================
// Cluster objects
// ============================================================

std::vector<Detection> clusterObjects(
    const CloudT::Ptr& cloud)
{
    std::vector<Detection> detections;

    if (cloud->empty())
        return detections;


    pcl::search::KdTree<PointT>::Ptr tree(
        new pcl::search::KdTree<PointT>);

    tree->setInputCloud(cloud);


    std::vector<pcl::PointIndices>
        clusterIndices;


    pcl::EuclideanClusterExtraction<PointT>
        ec;


    ec.setClusterTolerance(
        CLUSTER_TOLERANCE);

    ec.setMinClusterSize(
        MIN_CLUSTER_SIZE);

    ec.setMaxClusterSize(
        MAX_CLUSTER_SIZE);

    ec.setSearchMethod(tree);

    ec.setInputCloud(cloud);

    ec.extract(clusterIndices);


    for (const auto& indices :
         clusterIndices)
    {
        CloudT::Ptr cluster(
            new CloudT);

        cluster->reserve(
            indices.indices.size());


        for (int index :
             indices.indices)
        {
            cluster->push_back(
                (*cloud)[index]);
        }


        Eigen::Vector4f minPt;
        Eigen::Vector4f maxPt;

        pcl::getMinMax3D(
            *cluster,
            minPt,
            maxPt);


        float width =
            maxPt.x() - minPt.x();

        float depth =
            maxPt.y() - minPt.y();

        float height =
            maxPt.z() - minPt.z();


        Eigen::Vector4f center;

        center.x() =
            (minPt.x() + maxPt.x()) /
            2.0f;

        center.y() =
            (minPt.y() + maxPt.y()) /
            2.0f;

        center.z() =
            (minPt.z() + maxPt.z()) /
            2.0f;

        center.w() = 1.0f;


        // ----------------------------------------------------
        // Human geometry filtering
        // ----------------------------------------------------

        bool reasonableHeight =
            height >= MIN_HEIGHT &&
            height <= MAX_HEIGHT;


        bool reasonableWidth =
            width >= MIN_WIDTH &&
            width <= MAX_WIDTH;


        bool reasonableDepth =
            depth >= MIN_DEPTH &&
            depth <= MAX_DEPTH;


        bool reasonablePoints =
            indices.indices.size() >=
            MIN_PERSON_POINTS;


        if (reasonableHeight &&
            reasonableWidth &&
            reasonableDepth &&
            reasonablePoints)
        {
            Detection d;

            d.min_pt = minPt;
            d.max_pt = maxPt;

            d.center = center;

            d.width = width;
            d.depth = depth;
            d.height = height;

            d.points =
                indices.indices.size();


            detections.push_back(d);
        }
    }


    return detections;
}


// ============================================================
// Distance between detections
// ============================================================

float distance2D(
    const Eigen::Vector4f& a,
    const Eigen::Vector4f& b)
{
    float dx =
        a.x() - b.x();

    float dy =
        a.y() - b.y();

    return std::sqrt(
        dx * dx +
        dy * dy);
}


// ============================================================
// Tracker
// ============================================================
class PersonTracker
{
public:

    PersonTracker()
        : nextID(1)
    {
    }

    std::vector<Track>& getTracks()
    {
        return tracks;
    }

    void update(
        const std::vector<Detection>& detections,
        int frame)
    {
        const std::size_t oldTrackCount = tracks.size();

        std::vector<bool> detectionUsed(
            detections.size(), false);

        std::vector<bool> trackUsed(
            oldTrackCount, false);

        // ----------------------------------------------------
        // Associate detections with existing tracks
        // ----------------------------------------------------

        for (std::size_t t = 0;
             t < oldTrackCount;
             ++t)
        {
            float bestDistance =
                std::numeric_limits<float>::max();

            int bestDetection = -1;

            // Time since this track was last detected
            float dt =
                (frame - tracks[t].last_frame) *
                FRAME_TIME;

            if (dt <= 0.0f)
                dt = FRAME_TIME;

            // Predict where the object should be.
            Eigen::Vector4f predicted =
                tracks[t].center;

            if (tracks[t].moving)
            {
                predicted.x() +=
                    tracks[t].velocity.x() * dt;

                predicted.y() +=
                    tracks[t].velocity.y() * dt;
            }

            for (std::size_t d = 0;
                 d < detections.size();
                 ++d)
            {
                if (detectionUsed[d])
                    continue;

                float distance =
                    distance2D(
                        predicted,
                        detections[d].center);

                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestDetection =
                        static_cast<int>(d);
                }
            }

            // Allow a little more distance after missed frames.
            float allowedDistance =
                MAX_ASSOCIATION_DISTANCE;

            if (tracks[t].missed_frames > 0)
            {
                allowedDistance +=
                    0.25f *
                    static_cast<float>(
                        tracks[t].missed_frames);
            }

            // Prevent the search radius becoming excessive.
            allowedDistance =
                std::min(allowedDistance, 2.0f);

            if (bestDetection >= 0 &&
                bestDistance <= allowedDistance)
            {
                updateTrack(
                    tracks[t],
                    detections[bestDetection],
                    frame);

                detectionUsed[bestDetection] = true;
                trackUsed[t] = true;
            }
        }

        // ----------------------------------------------------
        // Create tracks for unmatched detections
        // ----------------------------------------------------

        for (std::size_t d = 0;
             d < detections.size();
             ++d)
        {
            if (detectionUsed[d])
                continue;

            Track track;

            track.id = nextID++;

            track.center =
                detections[d].center;
            track.start_center =
                detections[d].center;
            track.width =
                detections[d].width;

            track.depth =
                detections[d].depth;

            track.height =
                detections[d].height;

            track.points =
                detections[d].points;

            track.last_frame =
                frame;

            track.missed_frames = 0;

            track.speed = 0.0f;

            track.moving = false;

            track.velocity =
                Eigen::Vector4f::Zero();



            track.observations = 1;

            tracks.push_back(track);
        }

        // ----------------------------------------------------
        // Increase missed count
        // ----------------------------------------------------

        for (std::size_t i = 0;
             i < oldTrackCount;
             ++i)
        {
            if (!trackUsed[i])
            {
                tracks[i].missed_frames++;
            }
        }

        // ----------------------------------------------------
        // Remove tracks missing for too long
        // ----------------------------------------------------

        tracks.erase(
            std::remove_if(
                tracks.begin(),
                tracks.end(),
                [](const Track& t)
                {
                    return
                        t.missed_frames >
                        MAX_MISSED_FRAMES;
                }),
            tracks.end());
    }

private:

    int nextID;

    std::vector<Track> tracks;

    void updateTrack(
        Track& track,
        const Detection& detection,
        int frame)
    {
        float distance =
            distance2D(
                track.center,
                detection.center);

        int frameDifference =
            frame - track.last_frame;

        if (frameDifference <= 0)
            frameDifference = 1;

        float dt =
            static_cast<float>(
                frameDifference) *
            FRAME_TIME;

        // ----------------------------------------------------
        // Calculate velocity using the actual elapsed time
        // ----------------------------------------------------

        if (dt > 0.0f)
        {
            track.velocity.x() =
                (detection.center.x() -
                 track.center.x()) / dt;

            track.velocity.y() =
                (detection.center.y() -
                 track.center.y()) / dt;

            track.velocity.z() =
                (detection.center.z() -
                 track.center.z()) / dt;

            track.speed =
                distance / dt;
        }
        else
        {
            track.velocity =
                Eigen::Vector4f::Zero();

            track.speed = 0.0f;
        }

        // ----------------------------------------------------
        // Accumulate actual movement
        // ----------------------------------------------------

        track.observations++;

        float displacementFromStart =
            distance2D(
                track.start_center,
                detection.center);

        if (displacementFromStart >= MIN_MOVEMENT &&
            track.observations >= 3)
        {
            track.moving = true;
        }

        // ----------------------------------------------------
        // Update track
        // ----------------------------------------------------

        track.center =
            detection.center;

        track.width =
            detection.width;

        track.depth =
            detection.depth;

        track.height =
            detection.height;

        track.points =
            detection.points;

        track.last_frame =
            frame;

        track.missed_frames = 0;
    }
};



// ============================================================
// Main
// ============================================================

int main(
    int argc,
    char** argv)
{
    if (argc < 2)
    {
        std::cout
            << "\nUsage:\n"
            << "  "
            << argv[0]
            << " <pcd_directory> [output.csv]\n\n";

        std::cout
            << "Example:\n"
            << "  "
            << argv[0]
            << " ~/lidar_scans\n\n";

        return 1;
    }


    std::string directory =
        argv[1];


    std::string csvFile =
        argc >= 3
            ? argv[2]
            : "person_tracks.csv";


    if (!fs::exists(directory))
    {
        std::cerr
            << "Directory does not exist: "
            << directory
            << std::endl;

        return 1;
    }


    auto files =
        findPCDFiles(directory);


    if (files.empty())
    {
        std::cerr
            << "No PCD files found in: "
            << directory
            << std::endl;

        return 1;
    }


    std::cout
        << "\nFound "
        << files.size()
        << " PCD files.\n";


    std::cout
        << "First frame: "
        << getFrameNumber(files.front())
        << "\n";


    std::cout
        << "Last frame: "
        << getFrameNumber(files.back())
        << "\n";


    std::ofstream csv(csvFile);


    if (!csv)
    {
        std::cerr
            << "Could not create CSV: "
            << csvFile
            << std::endl;

        return 1;
    }


    csv
        << "frame,"
        << "file,"
        << "person_id,"
        << "x,"
        << "y,"
        << "z,"
        << "width,"
        << "depth,"
        << "height,"
        << "points,"
        << "speed,"
        << "moving\n";


    PersonTracker tracker;


    for (std::size_t fileIndex = 0;
         fileIndex < files.size();
         ++fileIndex)
    {
        const fs::path& filename =
            files[fileIndex];


        int frame =
            getFrameNumber(filename);


        std::cout
            << "\n----------------------------------------\n"
            << "Processing "
            << fileIndex + 1
            << "/"
            << files.size()
            << "\n"
            << "Frame: "
            << frame
            << "\n"
            << "File: "
            << filename.filename().string()
            << "\n";


        CloudT::Ptr cloud =
            loadCloud(
                filename.string());


        if (!cloud)
            continue;


        std::cout
            << "Original points: "
            << cloud->size()
            << "\n";


        CloudT::Ptr filtered =
            voxelFilter(cloud);


        std::cout
            << "After voxel filter: "
            << filtered->size()
            << "\n";


        CloudT::Ptr objects =
            removeGround(filtered);


        std::cout
            << "After ground removal: "
            << objects->size()
            << "\n";


        std::vector<Detection>
            detections =
                clusterObjects(objects);


        std::cout
            << "Person candidates: "
            << detections.size()
            << "\n";


        tracker.update(
            detections,
            frame);


        // ----------------------------------------------------
        // Output currently detected tracks
        // ----------------------------------------------------

        int visiblePeople = 0;


        for (const Track& track :
             tracker.getTracks())
        {
            if (track.last_frame != frame)
                continue;


            visiblePeople++;


            std::cout
                << std::fixed
                << std::setprecision(3);


            std::cout
                << "Person ID "
                << track.id
                << "\n";


            std::cout
                << "  Position: "
                << track.center.x()
                << ", "
                << track.center.y()
                << ", "
                << track.center.z()
                << "\n";


            std::cout
                << "  Size: "
                << track.width
                << " x "
                << track.depth
                << " x "
                << track.height
                << " m\n";


            std::cout
                << "  Points: "
                << track.points
                << "\n";


            std::cout
                << "  Speed: "
                << track.speed
                << " m/s\n";


            std::cout
                << "  Moving: "
                << (track.moving
                        ? "YES"
                        : "NO")
                << "\n";


            csv
                << frame
                << ","
                << filename.filename().string()
                << ","
                << track.id
                << ","
                << track.center.x()
                << ","
                << track.center.y()
                << ","
                << track.center.z()
                << ","
                << track.width
                << ","
                << track.depth
                << ","
                << track.height
                << ","
                << track.points
                << ","
                << track.speed
                << ","
                << (track.moving
                        ? 1
                        : 0)
                << "\n";
        }


        std::cout
            << "Visible tracks: "
            << visiblePeople
            << "\n";
    }


    csv.close();


    std::cout
        << "\n========================================\n"
        << "Finished.\n"
        << "Results:\n"
        << csvFile
        << "\n"
        << "========================================\n";


    return 0;
}

