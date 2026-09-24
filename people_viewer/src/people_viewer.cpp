#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

struct Result
{
    int frame = -1;
    int id = -1;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float width = 0.5f;
    float depth = 0.5f;
    float height = 1.7f;
    float speed = 0.0f;
    bool moving = false;
};

struct ViewerState
{
    bool paused = false;
    bool moving_only = false;
    int step = 0;
    bool quit = false;
};

static std::string trim(std::string s)
{
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front())))
        s.erase(s.begin());
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
        s.pop_back();
    return s;
}

static std::vector<std::string> splitCSV(const std::string& line)
{
    std::vector<std::string> out;
    std::string cur;
    bool quoted = false;

    for (char c : line)
    {
        if (c == '"')
        {
            quoted = !quoted;
        }
        else if (c == ',' && !quoted)
        {
            out.push_back(trim(cur));
            cur.clear();
        }
        else
        {
            cur += c;
        }
    }
    out.push_back(trim(cur));
    return out;
}

static std::string lower(std::string s)
{
    for (char &c : s)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

static int findColumn(const std::map<std::string, int>& cols,
                      std::initializer_list<const char*> names)
{
    for (const char* n : names)
    {
        auto it = cols.find(lower(n));
        if (it != cols.end())
            return it->second;
    }
    return -1;
}

static float toFloat(const std::vector<std::string>& row, int col, float fallback)
{
    if (col < 0 || col >= static_cast<int>(row.size()))
        return fallback;

    try
    {
        return std::stof(row[col]);
    }
    catch (...)
    {
        return fallback;
    }
}

static int toInt(const std::vector<std::string>& row, int col, int fallback)
{
    if (col < 0 || col >= static_cast<int>(row.size()))
        return fallback;

    try
    {
        return std::stoi(row[col]);
    }
    catch (...)
    {
        return fallback;
    }
}

static bool toBool(const std::vector<std::string>& row, int col, bool fallback)
{
    if (col < 0 || col >= static_cast<int>(row.size()))
        return fallback;

    std::string s = lower(trim(row[col]));
    return s == "1" || s == "true" || s == "yes" || s == "moving";
}

static std::map<int, std::vector<Result>>
loadResults(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file)
    {
        throw std::runtime_error("Cannot open CSV: " + filename);
    }

    std::string headerLine;
    if (!std::getline(file, headerLine))
        throw std::runtime_error("CSV is empty: " + filename);

    const auto headers = splitCSV(headerLine);

    std::map<std::string, int> cols;
    for (int i = 0; i < static_cast<int>(headers.size()); ++i)
        cols[lower(trim(headers[i]))] = i;

    const int cFrame  = findColumn(cols, {"frame", "frame_id"});
    const int cID     = findColumn(cols, {"person_id", "track_id", "id", "person"});
    const int cX      = findColumn(cols, {"center_x", "x", "cx"});
    const int cY      = findColumn(cols, {"center_y", "y", "cy"});
    const int cZ      = findColumn(cols, {"center_z", "z", "cz"});
    const int cMinX   = findColumn(cols, {"min_x"});
    const int cMaxX   = findColumn(cols, {"max_x"});
    const int cMinY   = findColumn(cols, {"min_y"});
    const int cMaxY   = findColumn(cols, {"max_y"});
    const int cMinZ   = findColumn(cols, {"min_z"});
    const int cMaxZ   = findColumn(cols, {"max_z"});
    const int cWidth  = findColumn(cols, {"width"});
    const int cDepth  = findColumn(cols, {"depth"});
    const int cHeight = findColumn(cols, {"height"});
    const int cSpeed  = findColumn(cols, {"speed", "speed_mps"});
    const int cMoving = findColumn(cols, {"moving", "is_moving"});

    if (cFrame < 0 || cID < 0 || cX < 0 || cY < 0)
    {
        std::cerr << "CSV columns found:\n";
        for (const auto& [name, idx] : cols)
            std::cerr << "  " << idx << ": " << name << "\n";

        throw std::runtime_error(
            "CSV must contain frame, person/track id, and x/y columns.");
    }

    std::map<int, std::vector<Result>> byFrame;

    std::string line;
    while (std::getline(file, line))
    {
        if (trim(line).empty())
            continue;

        const auto row = splitCSV(line);

        Result r;
        r.frame = toInt(row, cFrame, -1);
        r.id = toInt(row, cID, -1);

        r.x = toFloat(row, cX, 0.0f);
        r.y = toFloat(row, cY, 0.0f);

        bool haveCenterZ = cZ >= 0;
        r.z = toFloat(row, cZ, 0.0f);

        r.width = toFloat(row, cWidth, 0.5f);
        r.depth = toFloat(row, cDepth, 0.5f);
        r.height = toFloat(row, cHeight, 1.7f);

        r.speed = toFloat(row, cSpeed, 0.0f);
        r.moving = toBool(row, cMoving, false);

        // If min/max columns are available, use them to reconstruct
        // the exact box center/dimensions.
        if (cMinX >= 0 && cMaxX >= 0)
        {
            const float mn = toFloat(row, cMinX, r.x - r.width * 0.5f);
            const float mx = toFloat(row, cMaxX, r.x + r.width * 0.5f);
            r.x = 0.5f * (mn + mx);
            r.width = std::max(0.01f, mx - mn);
        }

        if (cMinY >= 0 && cMaxY >= 0)
        {
            const float mn = toFloat(row, cMinY, r.y - r.depth * 0.5f);
            const float mx = toFloat(row, cMaxY, r.y + r.depth * 0.5f);
            r.y = 0.5f * (mn + mx);
            r.depth = std::max(0.01f, mx - mn);
        }

        if (cMinZ >= 0 && cMaxZ >= 0)
        {
            const float mn = toFloat(row, cMinZ, 0.0f);
            const float mx = toFloat(row, cMaxZ, r.height);
            r.z = 0.5f * (mn + mx);
            r.height = std::max(0.01f, mx - mn);
        }
        else if (!haveCenterZ)
        {
            // No z in CSV: put the bottom of the person box at z=0.
            r.z = r.height * 0.5f;
        }

        if (r.frame >= 0 && r.id >= 0)
            byFrame[r.frame].push_back(r);
    }

    return byFrame;
}

static int frameNumber(const fs::path& p)
{
    const std::string stem = p.stem().string();
    const auto pos = stem.find_last_of('_');

    if (pos == std::string::npos)
        return -1;

    try
    {
        return std::stoi(stem.substr(pos + 1));
    }
    catch (...)
    {
        return -1;
    }
}

static std::vector<fs::path> findPCDs(const std::string& directory)
{
    std::vector<fs::path> files;

    for (const auto& e : fs::directory_iterator(directory))
    {
        if (!e.is_regular_file())
            continue;

        if (lower(e.path().extension().string()) == ".pcd")
            files.push_back(e.path());
    }

    std::sort(files.begin(), files.end(),
              [](const fs::path& a, const fs::path& b)
              {
                  return frameNumber(a) < frameNumber(b);
              });

    return files;
}

static void colorForID(int id, double& r, double& g, double& b)
{
    // A deterministic palette. IDs repeat after the palette length.
    static const double palette[][3] =
    {
        {1.0, 0.2, 0.2},
        {0.2, 1.0, 0.2},
        {0.2, 0.5, 1.0},
        {1.0, 0.8, 0.1},
        {0.9, 0.2, 1.0},
        {0.1, 0.9, 0.9},
        {1.0, 0.5, 0.1},
        {0.6, 0.3, 1.0},
        {0.8, 1.0, 0.2},
        {1.0, 0.3, 0.7}
    };

    constexpr int N = static_cast<int>(sizeof(palette) / sizeof(palette[0]));
    const int i = std::abs(id) % N;

    r = palette[i][0];
    g = palette[i][1];
    b = palette[i][2];
}

static void clearObjects(pcl::visualization::PCLVisualizer::Ptr viewer)
{
    // Remove all known box/text IDs. The point cloud itself has ID "cloud".
    for (int id = 0; id < 10000; ++id)
    {
        viewer->removeShape("box_" + std::to_string(id));
        viewer->removeShape("text_" + std::to_string(id));
    }

    viewer->removeShape("frame_text");
    viewer->removeShape("info_text");
}

static void drawResults(
    pcl::visualization::PCLVisualizer::Ptr viewer,
    const std::vector<Result>& results,
    bool movingOnly)
{
    int labelIndex = 0;

    for (const Result& r : results)
    {
        if (movingOnly && !r.moving)
            continue;

        double cr, cg, cb;
        colorForID(r.id, cr, cg, cb);

        const double xmin = r.x - 0.5 * r.width;
        const double xmax = r.x + 0.5 * r.width;
        const double ymin = r.y - 0.5 * r.depth;
        const double ymax = r.y + 0.5 * r.depth;
        const double zmin = r.z - 0.5 * r.height;
        const double zmax = r.z + 0.5 * r.height;

        const std::string boxID = "box_" + std::to_string(labelIndex);
        const std::string textID = "text_" + std::to_string(labelIndex);

        viewer->addCube(
            xmin, xmax,
            ymin, ymax,
            zmin, zmax,
            cr, cg, cb,
            boxID);

        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
            boxID);

        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
            3.0,
            boxID);

        PointT textPoint;
        textPoint.x = static_cast<float>(r.x);
        textPoint.y = static_cast<float>(r.y);
        textPoint.z = static_cast<float>(zmax + 0.15);

        std::ostringstream label;
        label << "ID " << r.id;
        label << (r.moving ? "  MOVING" : "  STATIC");
        label << "  " << std::fixed << std::setprecision(2)
              << r.speed << " m/s";

        viewer->addText3D(
            label.str(),
            textPoint,
            0.15,
            cr, cg, cb,
            textID);

        ++labelIndex;
    }
}

static void keyboardCallback(
    const pcl::visualization::KeyboardEvent& event,
    void* cookie)
{
    if (!event.keyDown())
        return;

    auto* state = static_cast<ViewerState*>(cookie);

    const std::string key = event.getKeySym();

    if (key == "space")
    {
        state->paused = !state->paused;
        std::cout << (state->paused ? "Paused\n" : "Playing\n");
    }
    else if (key == "n" || key == "N")
    {
        if (state->paused)
            state->step = 1;
    }
    else if (key == "b" || key == "B")
    {
        if (state->paused)
            state->step = -1;
    }
    else if (key == "m" || key == "M")
    {
        state->moving_only = !state->moving_only;
        std::cout << "Moving-only display: "
                  << (state->moving_only ? "ON" : "OFF") << "\n";
    }
    else if (key == "q" || key == "Q" || key == "Escape")
    {
        state->quit = true;
    }
}

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr
            << "Usage:\n  " << argv[0]
            << " <pcd_directory> <results.csv>\n\n"
            << "Example:\n  " << argv[0]
            << " ~/Documents/rslidar_pcds "
            << "\"$HOME/Documents/rslidar_people_detector/test_results(5).csv\"\n";
        return 1;
    }

    const std::string pcdDir = argv[1];
    const std::string csvFile = argv[2];

    if (!fs::exists(pcdDir))
    {
        std::cerr << "PCD directory does not exist: " << pcdDir << "\n";
        return 1;
    }

    if (!fs::exists(csvFile))
    {
        std::cerr << "CSV does not exist: " << csvFile << "\n";
        return 1;
    }

    std::vector<fs::path> pcds;
    try
    {
        pcds = findPCDs(pcdDir);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Could not scan PCD directory: " << e.what() << "\n";
        return 1;
    }

    if (pcds.empty())
    {
        std::cerr << "No PCD files found in: " << pcdDir << "\n";
        return 1;
    }

    std::map<int, std::vector<Result>> results;
    try
    {
        results = loadResults(csvFile);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Could not load CSV: " << e.what() << "\n";
        return 1;
    }

    std::cout << "PCD frames: " << pcds.size() << "\n";
    std::cout << "CSV frames with results: " << results.size() << "\n";
    std::cout << "Controls: SPACE pause, N next, B previous, M moving-only, Q quit\n";

    auto viewer = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer("RSLiDAR People Tracking"));

    viewer->setBackgroundColor(0.04, 0.04, 0.04);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    ViewerState state;
    viewer->registerKeyboardCallback(keyboardCallback, &state);

    int index = 0;

    while (!viewer->wasStopped() && !state.quit)
    {
        if (index < 0)
            index = 0;
        if (index >= static_cast<int>(pcds.size()))
            index = static_cast<int>(pcds.size()) - 1;

        const int currentFrame = frameNumber(pcds[index]);

        CloudT::Ptr cloud(new CloudT);
        if (pcl::io::loadPCDFile<PointT>(pcds[index].string(), *cloud) != 0)
        {
            std::cerr << "Could not load " << pcds[index] << "\n";
            ++index;
            continue;
        }

        viewer->removePointCloud("cloud");
        viewer->addPointCloud<PointT>(cloud, "cloud");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            2.0,
            "cloud");

        clearObjects(viewer);

        auto it = results.find(currentFrame);
        const std::vector<Result>* frameResults = nullptr;

        if (it != results.end())
            frameResults = &it->second;

        if (frameResults)
            drawResults(viewer, *frameResults, state.moving_only);

        std::ostringstream info;
        info << "Frame: " << currentFrame
             << "  [" << (index + 1) << "/" << pcds.size() << "]";

        if (frameResults)
        {
            int shown = 0;
            for (const auto& r : *frameResults)
                if (!state.moving_only || r.moving)
                    ++shown;

            info << "  People: " << shown;
        }
        else
        {
            info << "  People: 0";
        }

        info << "  | SPACE pause/play | N/B step | M moving-only | Q quit";

        viewer->addText(
            info.str(),
            10, 10,
            1.0, 1.0, 1.0,
            "info_text");

        viewer->spinOnce(10);

        if (state.quit)
            break;

        if (state.paused)
        {
            if (state.step != 0)
            {
                index += state.step;
                state.step = 0;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        else
        {
            ++index;

            // Approx. 10 Hz, matching the detector's FRAME_TIME=0.1.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (index >= static_cast<int>(pcds.size()) && !state.paused)
        {
            // Stop at the final frame and pause.
            index = static_cast<int>(pcds.size()) - 1;
            state.paused = true;
            std::cout << "Reached final frame. Press B/N or SPACE.\n";
        }
    }

    return 0;
}
