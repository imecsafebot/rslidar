#include "detectoradapter.h"

#include <QMetaType>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>

DetectorAdapter* DetectorAdapter::s_instance = nullptr;

namespace
{
double nowSeconds()
{
    using Clock = std::chrono::steady_clock;
    static const auto start = Clock::now();
    return std::chrono::duration<double>(Clock::now() - start).count();
}
}

DetectorAdapter::DetectorAdapter(QObject* parent)
    : QThread(parent)
{
    qRegisterMetaType<DetectionFrame>(
        "DetectionFrame");

    m_currentDetectorConfig =
        PeopleDetector::Config{};

    m_currentTrackerConfig =
        PeopleTracker::Config{};

    m_pendingDetectorConfig =
        m_currentDetectorConfig;

    m_pendingTrackerConfig =
        m_currentTrackerConfig;
}

void DetectorAdapter::setDetectorConfig(
    const PeopleDetector::Config& config)
{
    {
        std::lock_guard<std::mutex> lock(m_configMutex);
        m_pendingDetectorConfig = config;
    }

    m_configDirty = true;
}

void DetectorAdapter::setTrackerConfig(
    const PeopleTracker::Config& config)
{
    {
        std::lock_guard<std::mutex> lock(m_configMutex);
        m_pendingTrackerConfig = config;
    }

    m_configDirty = true;
}

PeopleDetector::Config
DetectorAdapter::detectorConfig() const
{
    std::lock_guard<std::mutex> lock(m_configMutex);
    return m_currentDetectorConfig;
}

PeopleTracker::Config
DetectorAdapter::trackerConfig() const
{
    std::lock_guard<std::mutex> lock(m_configMutex);
    return m_currentTrackerConfig;
}

void DetectorAdapter::applyPendingConfiguration()
{
    if (!m_configDirty)
        return;

    PeopleDetector::Config detectorConfig;
    PeopleTracker::Config trackerConfig;

    {
        std::lock_guard<std::mutex> lock(m_configMutex);

        detectorConfig = m_pendingDetectorConfig;
        trackerConfig = m_pendingTrackerConfig;

        m_currentDetectorConfig = detectorConfig;
        m_currentTrackerConfig = trackerConfig;

        m_configDirty = false;
    }

    m_detector.setConfig(detectorConfig);
    m_tracker.setConfig(trackerConfig);

    // Start tracking again using the new parameters.
    m_tracker.reset();

    emit configurationApplied();
}



DetectorAdapter::~DetectorAdapter()
{
    stopDetection();
    wait(3000);
}

std::shared_ptr<RSLidarCloudMsg> DetectorAdapter::getPointCloud()
{
    return std::make_shared<RSLidarCloudMsg>();
}

void DetectorAdapter::returnPointCloud(std::shared_ptr<RSLidarCloudMsg> msg)
{
    if (!s_instance || !s_instance->m_running)
        return;

    {
        std::lock_guard<std::mutex> lock(s_instance->m_queueMutex);

        while (s_instance->m_queue.size() >= MAX_QUEUE_SIZE)
            s_instance->m_queue.pop_front();

        s_instance->m_queue.push_back(std::move(msg));
    }

    s_instance->m_queueCv.notify_one();
}

void DetectorAdapter::exceptionCallback(const robosense::lidar::Error& error)
{
    if (s_instance)
        emit s_instance->errorOccurred(
            QString::fromStdString(error.toString()));
}

void DetectorAdapter::clearQueue()
{
    std::lock_guard<std::mutex> lock(m_queueMutex);
    m_queue.clear();
}

std::shared_ptr<RSLidarCloudMsg> DetectorAdapter::getNextCloud()
{
    std::unique_lock<std::mutex> lock(m_queueMutex);

    m_queueCv.wait(lock, [this]
    {
        return !m_running || !m_queue.empty();
    });

    if (!m_running || m_queue.empty())
        return nullptr;

    auto msg = std::move(m_queue.back());
    m_queue.clear();
    return msg;
}

void DetectorAdapter::startDetection()
{
    if (isRunning())
        return;

    m_running = true;
    start();
}

void DetectorAdapter::stopDetection()
{
    m_running = false;
    m_queueCv.notify_all();

    // The worker owns the driver and stops it after leaving its loop.
}

void DetectorAdapter::run()
{
    s_instance = this;
    clearQueue();
    m_detector.setConfig(
        m_currentDetectorConfig);

    m_tracker.setConfig(
        m_currentTrackerConfig);

    m_tracker.reset();
        emit statusChanged("CONNECTING");

    robosense::lidar::RSDriverParam param;
    param.input_type = robosense::lidar::InputType::ONLINE_LIDAR;
    param.input_param.msop_port = 6699;
    param.input_param.difop_port = 7788;
    param.lidar_type = robosense::lidar::LidarType::RSE1;

    m_driver = std::make_unique<robosense::lidar::LidarDriver<RSLidarCloudMsg>>();
    m_driver->regPointCloudCallback(getPointCloud, returnPointCloud);
    m_driver->regExceptionCallback(exceptionCallback);

    if (!m_driver->init(param))
    {
        emit errorOccurred("Failed to initialize the RSLiDAR E1R driver.");
        m_running = false;
        m_driver.reset();
        s_instance = nullptr;
        return;
    }

    if (!m_driver->start())
    {
        emit errorOccurred("Failed to start the RSLiDAR E1R driver.");
        m_running = false;
        m_driver.reset();
        s_instance = nullptr;
        return;
    }

    emit statusChanged("CONNECTED");

    auto fpsStart = std::chrono::steady_clock::now();
    std::uint64_t fpsFrames = 0;
    double displayFps = 0.0;

    while (m_running)
    {
        applyPendingConfiguration();

        auto msg = getNextCloud();
        if (!msg)
            break;

        const auto processStart = std::chrono::steady_clock::now();

        auto input = std::make_shared<InputCloud>();
        input->reserve(msg->points.size());

        for (const auto& p : msg->points)
        {
            if (!std::isfinite(p.x) ||
                !std::isfinite(p.y) ||
                !std::isfinite(p.z))
                continue;

            if (p.x == 0.0f && p.y == 0.0f && p.z == 0.0f)
                continue;

            pcl::PointXYZI pcl_point;
        pcl_point.x = p.x;
        pcl_point.y = p.y;
        pcl_point.z = p.z;
        pcl_point.intensity = static_cast<float>(p.intensity);

        input->push_back(pcl_point);
        }

        input->width = static_cast<std::uint32_t>(input->size());
        input->height = 1;
        input->is_dense = false;

        if (std::getenv("PEOPLE_PROBE_TRACE") && msg->seq % 10 == 0) {
            std::ofstream raw("probe_raw.csv"); raw << "x,y,z\n";
            for (const auto& p : input->points) raw << p.x << ',' << p.y << ',' << p.z << '\n';
        }

        InputCloud::Ptr visualizationCloud;
        PeopleDetector::Diagnostics diagnostics;
        auto detections = m_detector.detect(input, visualizationCloud, &diagnostics);
        m_tracker.update(detections, nowSeconds());

        const auto processEnd = std::chrono::steady_clock::now();
        const double processingMs =
            std::chrono::duration<double, std::milli>(
                processEnd - processStart).count();

        ++fpsFrames;
        const auto fpsNow = std::chrono::steady_clock::now();
        const double fpsElapsed =
            std::chrono::duration<double>(fpsNow - fpsStart).count();

        if (fpsElapsed >= 1.0)
        {
            displayFps = static_cast<double>(fpsFrames) / fpsElapsed;
            fpsFrames = 0;
            fpsStart = fpsNow;
        }

        DetectionFrame frame;
        frame.diagnostics = diagnostics;
        frame.fps = displayFps;
        frame.processingMs = processingMs;
        frame.frameNumber = msg->seq;

        if (visualizationCloud)
        {
            frame.points.reserve(static_cast<int>(visualizationCloud->size()));
            for (const auto& p : visualizationCloud->points)
                frame.points.push_back({p.x, p.y, p.z});
        }

        frame.people.reserve(static_cast<int>(detections.size()));
        for (const auto& d : detections)
        {
            const float distance = std::sqrt(
                d.center.x() * d.center.x() +
                d.center.y() * d.center.y() +
                d.center.z() * d.center.z());

            frame.people.push_back({
                d.id,
                d.center.x(), d.center.y(), d.center.z(),
                d.width, d.depth, d.height,
                distance, d.speed, d.moving,
                static_cast<int>(d.points)
            });
        }

        emit frameReady(frame);
    }

    if (m_driver)
        m_driver->stop();

    m_driver.reset();
    clearQueue();
    m_running = false;

    emit statusChanged("STOPPED");
    s_instance = nullptr;
}
