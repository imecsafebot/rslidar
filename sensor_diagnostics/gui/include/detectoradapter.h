#pragma once

#include "people_detector.h"
#include "people_tracker.h"

#include <QThread>
#include <QVector>
#include <QString>
#include <QMetaType>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>

using RSLidarPoint = PointXYZI;
using RSLidarCloudMsg = PointCloudT<RSLidarPoint>;

struct GuiPoint
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct GuiPerson
{
    int id = -1;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    float width = 0.0f;
    float depth = 0.0f;
    float height = 0.0f;

    float distance = 0.0f;
    float speed = 0.0f;

    bool moving = false;
    int points = 0;
};

struct DetectionFrame
{
    PeopleDetector::Diagnostics diagnostics;
    QVector<GuiPoint> points;
    QVector<GuiPerson> people;

    double fps = 0.0;
    double processingMs = 0.0;

    std::uint64_t frameNumber = 0;
};

Q_DECLARE_METATYPE(DetectionFrame)


class DetectorAdapter : public QThread
{
    Q_OBJECT

public:
    explicit DetectorAdapter(QObject* parent = nullptr);
    ~DetectorAdapter() override;

    void startDetection();
    void stopDetection();

    void setDetectorConfig(
        const PeopleDetector::Config& config);

    void setTrackerConfig(
        const PeopleTracker::Config& config);

    PeopleDetector::Config detectorConfig() const;

    PeopleTracker::Config trackerConfig() const;

signals:
    void frameReady(const DetectionFrame& frame);
    void statusChanged(const QString& status);
    void errorOccurred(const QString& message);
    void configurationApplied();

protected:
    void run() override;

private:
    using Driver =
        robosense::lidar::LidarDriver<RSLidarCloudMsg>;

    static std::shared_ptr<RSLidarCloudMsg> getPointCloud();

    static void returnPointCloud(
        std::shared_ptr<RSLidarCloudMsg> msg);

    static void exceptionCallback(
        const robosense::lidar::Error& error);

    std::shared_ptr<RSLidarCloudMsg> getNextCloud();

    void clearQueue();

    void applyPendingConfiguration();

    std::atomic_bool m_running{false};

    std::mutex m_queueMutex;
    std::condition_variable m_queueCv;

    std::deque<
        std::shared_ptr<RSLidarCloudMsg>
    > m_queue;

    std::unique_ptr<Driver> m_driver;

    PeopleDetector m_detector;
    PeopleTracker m_tracker;

    mutable std::mutex m_configMutex;

    PeopleDetector::Config m_pendingDetectorConfig;
    PeopleTracker::Config m_pendingTrackerConfig;

    PeopleDetector::Config m_currentDetectorConfig;
    PeopleTracker::Config m_currentTrackerConfig;

    std::atomic_bool m_configDirty{false};

    static DetectorAdapter* s_instance;

    static constexpr std::size_t MAX_QUEUE_SIZE = 2;
};
