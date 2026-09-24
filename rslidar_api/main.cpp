#include <atomic>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rs_driver/api/lidar_driver.hpp"
#include "rs_driver/msg/point_cloud_msg.hpp"
#include "rs_driver/utility/sync_queue.hpp"

using namespace robosense::lidar;

using PointT = PointXYZI;
using PointCloudMsg = PointCloudT<PointT>;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

std::atomic<bool> running(true);

void signalHandler(int)
{
    running = false;
}

std::shared_ptr<PointCloudMsg> getPointCloud()
{
    auto msg = free_cloud_queue.pop();

    if (msg)
        return msg;

    return std::make_shared<PointCloudMsg>();
}

void returnPointCloud(std::shared_ptr<PointCloudMsg> msg)
{
    stuffed_cloud_queue.push(msg);
}

void exceptionCallback(const Error& error)
{
    std::cerr << "Driver error: "
              << error.toString()
              << std::endl;
}

bool savePCD(const std::string& filename,
             const std::shared_ptr<PointCloudMsg>& cloud)
{
    std::ofstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "ERROR: Cannot open "
                  << filename
                  << " for writing."
                  << std::endl;
        return false;
    }

    size_t valid_points = 0;

    for (const auto& p : cloud->points)
    {
        if (p.x != 0.0f ||
            p.y != 0.0f ||
            p.z != 0.0f)
        {
            ++valid_points;
        }
    }

    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z intensity\n";
    file << "SIZE 4 4 4 1\n";
    file << "TYPE F F F U\n";
    file << "COUNT 1 1 1 1\n";
    file << "WIDTH " << valid_points << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << valid_points << "\n";
    file << "DATA ascii\n";

    file << std::fixed << std::setprecision(6);

    for (const auto& p : cloud->points)
    {
        if (p.x == 0.0f &&
            p.y == 0.0f &&
            p.z == 0.0f)
        {
            continue;
        }

        file << p.x << " "
             << p.y << " "
             << p.z << " "
             << static_cast<unsigned int>(p.intensity)
             << "\n";
    }

    file.close();

    std::cout << "Saved PCD: "
              << filename
              << " (" << valid_points
              << " valid points)"
              << std::endl;

    return true;
}

void processCloud()
{
    unsigned int saved_frames = 0;

    while (running)
    {
        auto cloud = stuffed_cloud_queue.popWait();

        if (!cloud)
            continue;

        size_t valid_points = 0;

        for (const auto& p : cloud->points)
        {
            if (p.x != 0.0f ||
                p.y != 0.0f ||
                p.z != 0.0f)
            {
                ++valid_points;
            }
        }

        std::cout << "\n========================================\n";
        std::cout << "Frame: " << cloud->seq << "\n";
        std::cout << "Total points: "
                  << cloud->points.size() << "\n";
        std::cout << "Valid points: "
                  << valid_points << "\n";

        if (!cloud->points.empty())
        {
            size_t printed = 0;

            for (size_t i = 0;
                 i < cloud->points.size() && printed < 5;
                 ++i)
            {
                const auto& p = cloud->points[i];

                if (p.x == 0.0f &&
                    p.y == 0.0f &&
                    p.z == 0.0f)
                {
                    continue;
                }

                std::cout << std::fixed
                          << std::setprecision(6)
                          << "Point " << i
                          << ": X=" << p.x
                          << ", Y=" << p.y
                          << ", Z=" << p.z
                          << ", Intensity="
                          << static_cast<unsigned int>(p.intensity)
                          << "\n";

                ++printed;
            }
        }

        std::cout << "========================================\n";

        // Save every received frame as an individual PCD file.
        std::string filename =
            "scan_frame_" +
            std::to_string(saved_frames) +
            ".pcd";

        savePCD(filename, cloud);

        ++saved_frames;

        // Return the cloud object to the SDK for reuse.
        free_cloud_queue.push(cloud);
    }
}

int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "========================================\n";
    std::cout << " RoboSense E1R Standalone SDK API\n";
    std::cout << "========================================\n";

    RSDriverParam param;

    // E1R receives point-cloud data directly from the Ethernet/UDP sensor.
    param.input_type = InputType::ONLINE_LIDAR;

    // E1R network ports.
    param.input_param.msop_port = 6699;
    param.input_param.difop_port = 7788;

    // Select the E1R decoder.
    param.lidar_type = LidarType::RSE1;

    param.print();

    LidarDriver<PointCloudMsg> driver;

    driver.regPointCloudCallback(
        getPointCloud,
        returnPointCloud);

    driver.regExceptionCallback(
        exceptionCallback);

    if (!driver.init(param))
    {
        std::cerr
            << "ERROR: Failed to initialize RoboSense E1R driver."
            << std::endl;

        return 1;
    }

    std::thread cloud_thread(processCloud);

    if (!driver.start())
    {
        std::cerr
            << "ERROR: Failed to start RoboSense E1R driver."
            << std::endl;

        running = false;
        cloud_thread.join();

        return 1;
    }

    std::cout << "\nRoboSense E1R SDK is running.\n";
    std::cout << "Receiving UDP point clouds on MSOP port 6699.\n";
    std::cout << "Press Ctrl+C to stop.\n\n";

    while (running)
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(200));
    }

    std::cout << "\nStopping E1R driver...\n";

    driver.stop();

    // Wake the processing thread if it is waiting for another cloud.
    stuffed_cloud_queue.push(nullptr);

    if (cloud_thread.joinable())
        cloud_thread.join();

    std::cout << "E1R SDK stopped.\n";

    return 0;
}
