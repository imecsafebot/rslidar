#include "mainwindow.h"

#include <QAbstractItemView>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QScrollArea>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QPlainTextEdit>
#include <QDateTime>
#include <QDir>
#include <QStandardPaths>
#include <QJsonObject>
#include <QJsonDocument>
#include <QFileInfo>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("RSLiDAR E1R — People Detection");
    resize(1450, 900);
    buildUi();

    m_detector = new DetectorAdapter(this);

    connect(m_detector, &DetectorAdapter::frameReady,
            this, &MainWindow::updateFrame);
    connect(m_detector, &DetectorAdapter::statusChanged,
            this, &MainWindow::updateStatus);
    connect(m_detector, &DetectorAdapter::errorOccurred,
            this, &MainWindow::showError);

    connect(m_start, &QPushButton::clicked,
            this, &MainWindow::startDetection);
    connect(m_stop, &QPushButton::clicked,
            this, &MainWindow::stopDetection);
    connect(m_reset, &QPushButton::clicked,
            this, &MainWindow::resetView);

    loadParameters();
    setRunningUi(false);
}

void MainWindow::buildUi()
{
    auto* central = new QWidget(this);
    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(14, 12, 14, 12);
    root->setSpacing(10);

    auto* title = new QLabel("RSLiDAR E1R  |  REAL-TIME PEOPLE DETECTION");
    title->setStyleSheet("font-size:22px;font-weight:700;padding:4px 0 8px;");
    root->addWidget(title);

    auto* metrics = new QHBoxLayout;
    m_connection = new QLabel("● DISCONNECTED");
    m_runState = new QLabel("STOPPED");
    m_people = new QLabel("People: 0");
    m_fps = new QLabel("FPS: 0.0");
    m_latency = new QLabel("Latency: 0.0 ms");
    m_frame = new QLabel("Frame: —");

    for (auto* label : {m_connection, m_runState, m_people,
                        m_fps, m_latency, m_frame})
    {
        label->setMinimumHeight(36);
        label->setAlignment(Qt::AlignCenter);
        label->setStyleSheet(
            "background:#20252d;border-radius:6px;padding:5px 12px;");
        metrics->addWidget(label);
    }
    root->addLayout(metrics);

    auto* body = new QHBoxLayout;
    body->setSpacing(10);

    m_view = new PointCloudWidget;
    body->addWidget(m_view, 1);

    auto* side = new QVBoxLayout;
    side->setSpacing(10);

    auto* controls = new QGroupBox("CONTROL");
    auto* controlsLayout = new QVBoxLayout(controls);
    m_start = new QPushButton("START DETECTION");
    m_stop = new QPushButton("STOP");
    m_reset = new QPushButton("RESET VIEW");
    m_start->setMinimumHeight(44);
    m_stop->setMinimumHeight(44);
    m_reset->setMinimumHeight(38);
    controlsLayout->addWidget(m_start);
    controlsLayout->addWidget(m_stop);
    controlsLayout->addWidget(m_reset);
    side->addWidget(controls);


    auto* parameterBox = new QGroupBox("DETECTION PARAMETERS");
    auto* parameterOuter = new QVBoxLayout(parameterBox);
    auto* parameterScroll = new QScrollArea;
    parameterScroll->setWidgetResizable(true);
    parameterScroll->setMinimumHeight(270);
    auto* parameterContent = new QWidget;
    auto* parameterForm = new QFormLayout(parameterContent);
    auto addFloat = [&](const QString& key, const QString& label, double lo, double hi, double step) {
        auto* spin = new QDoubleSpinBox;
        spin->setRange(lo, hi);
        spin->setDecimals(3);
        spin->setSingleStep(step);
        spin->setSuffix("  ");
        m_floats.insert(key, spin);
        parameterForm->addRow(label, spin);
    };
    auto addInt = [&](const QString& key, const QString& label, int lo, int hi) {
        auto* spin = new QSpinBox;
        spin->setRange(lo, hi);
        m_ints.insert(key, spin);
        parameterForm->addRow(label, spin);
    };
    auto addBool = [&](const QString& key, const QString& label) {
        auto* check = new QCheckBox;
        m_bools.insert(key, check);
        parameterForm->addRow(label, check);
    };
    addFloat("voxel_size", "Voxel size (m)", .005, .3, .005);
    addFloat("ground_distance", "Ground distance (m)", .01, .5, .01);
    addInt("ground_iterations", "Ground iterations", 10, 2000);
    addFloat("cluster_tolerance", "Cluster tolerance (m)", .05, 1.5, .01);
    addInt("min_cluster_size", "Min cluster points", 1, 100000);
    addInt("max_cluster_size", "Max cluster points", 1, 1000000);
    addFloat("min_height", "Min height (m)", .1, 4, .05);
    addFloat("max_height", "Max height (m)", .1, 4, .05);
    addFloat("min_width", "Min width (m)", .01, 3, .05);
    addFloat("max_width", "Max width (m)", .01, 3, .05);
    addFloat("min_depth", "Min depth (m)", .01, 3, .05);
    addFloat("max_depth", "Max depth (m)", .01, 3, .05);
    addInt("min_person_points", "Min person points", 1, 100000);
    addFloat("min_range", "Min range (m)", 0, 100, .1);
    addFloat("max_range", "Max range (m)", .1, 100, .1);
    addBool("enable_edge_rejection", "Reject FOV edges");
    addFloat("max_horizontal_angle_deg", "Half FOV (degrees)", 1, 180, 5);
    addBool("enable_flat_object_rejection", "Reject flat objects");
    addFloat("max_flatness", "Max flatness", 0, 1, .05);
    addFloat("max_association_distance", "Association distance (m)", .05, 10, .1);
    addFloat("max_prediction_distance", "Prediction distance (m)", .05, 10, .1);
    addInt("max_missed_frames", "Max missed frames", 0, 1000);
    addFloat("min_movement", "Movement threshold (m)", 0, 10, .05);
    addFloat("fallback_dt", "Fallback frame dt (s)", .001, 5, .01);
    parameterScroll->setWidget(parameterContent);
    parameterOuter->addWidget(parameterScroll);
    m_apply = new QPushButton("APPLY PARAMETERS");
    parameterOuter->addWidget(m_apply);
    connect(m_apply, &QPushButton::clicked, this, &MainWindow::applyParameters);
    side->addWidget(parameterBox);

    auto* peopleBox = new QGroupBox("DETECTED PEOPLE");
    auto* peopleLayout = new QVBoxLayout(peopleBox);
    m_table = new QTableWidget(0, 6);
    m_table->setHorizontalHeaderLabels(
        {"ID", "Distance", "Height", "Speed", "Points", "State"});
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_table->verticalHeader()->setVisible(false);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setSelectionMode(QAbstractItemView::NoSelection);
    peopleLayout->addWidget(m_table);
    side->addWidget(peopleBox, 1);

    auto* diagnosticsBox = new QGroupBox("DETECTION PIPELINE DIAGNOSTICS");
    auto* diagnosticsLayout = new QVBoxLayout(diagnosticsBox);
    m_pipelineSummary = new QLabel("Waiting for LiDAR frames...");
    m_pipelineSummary->setWordWrap(true);
    m_pipelineSummary->setTextInteractionFlags(Qt::TextSelectableByMouse);
    diagnosticsLayout->addWidget(m_pipelineSummary);
    side->addWidget(diagnosticsBox);

    auto* logBox = new QGroupBox("RESULTS / LIVE LOG");
    auto* logLayout = new QVBoxLayout(logBox);
    m_logPath = new QLabel("Log starts automatically with detection");
    m_logPath->setWordWrap(true);
    logLayout->addWidget(m_logPath);
    m_logPreview = new QPlainTextEdit;
    m_logPreview->setReadOnly(true);
    m_logPreview->setMaximumBlockCount(250);
    m_logPreview->setMinimumHeight(130);
    logLayout->addWidget(m_logPreview);
    side->addWidget(logBox);
    body->addLayout(side, 0);
    root->addLayout(body, 1);
    setCentralWidget(central);

    setStyleSheet(
        "QMainWindow,QWidget{background:#11151b;color:#e8edf2;}"
        "QGroupBox{border:1px solid #343b46;border-radius:7px;"
        "margin-top:10px;padding:10px;font-weight:600;}"
        "QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 5px;}"
        "QPushButton{background:#2b333e;border:1px solid #46505d;"
        "border-radius:6px;padding:8px;font-weight:600;}"
        "QPushButton:hover{background:#37414e;}"
        "QPushButton:disabled{color:#6f7884;background:#20252d;}"
        "QTableWidget{background:#171c23;border:1px solid #343b46;}"
        "QHeaderView::section{background:#252c35;padding:6px;border:0;}");
}

void MainWindow::setRunningUi(bool running)
{
    m_start->setEnabled(!running);
    m_stop->setEnabled(running);
    m_runState->setText(running ? "RUNNING" : "STOPPED");
}

void MainWindow::startDetection()
{
    applyParameters();
    beginLog();
    m_detector->startDetection();
    setRunningUi(true);
}

void MainWindow::stopDetection()
{
    m_detector->stopDetection();
    endLog();
    setRunningUi(false);
    m_connection->setText("● STOPPED");
}

void MainWindow::resetView()
{
    m_view->resetView();
}

void MainWindow::updateStatus(const QString& status)
{
    if (status == "CONNECTED")
        m_connection->setText("● CONNECTED");
    else if (status == "STOPPED")
        m_connection->setText("● STOPPED");
    else
        m_connection->setText("● " + status);
}

void MainWindow::showError(const QString& message)
{
    setRunningUi(false);
    endLog();
    m_connection->setText("● ERROR");
    QMessageBox::critical(this, "RSLiDAR Error", message);
}

void MainWindow::updateFrame(const DetectionFrame& frame)
{
    m_view->setFrame(frame);
    appendLog(frame);
    const auto& d = frame.diagnostics;
    m_pipelineSummary->setText(QString(
        "Raw %1  |  ROI %2  |  Voxel %3  |  Non-ground %4\n"
        "Clusters %5  |  Accepted %6\n"
        "Rejected: points %7, height %8, width %9, depth %10, "
        "range %11, angle %12, flatness %13")
        .arg(d.raw_points).arg(d.roi_points).arg(d.voxel_points)
        .arg(d.non_ground_points).arg(d.clusters).arg(d.accepted)
        .arg(d.rejected_points).arg(d.rejected_height)
        .arg(d.rejected_width).arg(d.rejected_depth)
        .arg(d.rejected_range).arg(d.rejected_angle)
        .arg(d.rejected_flatness));
    if (m_pipelineCsv.isOpen()) {
        m_pipelineStream << QDateTime::currentDateTime().toString(Qt::ISODateWithMs)
            << ',' << frame.frameNumber << ',' << d.raw_points << ','
            << d.roi_points << ',' << d.voxel_points << ','
            << d.ground_removed_points << ',' << d.non_ground_points << ','
            << d.clusters << ',' << d.rejected_points << ','
            << d.rejected_height << ',' << d.rejected_width << ','
            << d.rejected_depth << ',' << d.rejected_range << ','
            << d.rejected_angle << ',' << d.rejected_flatness << ','
            << d.accepted << '\n';
        m_pipelineStream.flush();
    }
    m_people->setText(QString("People: %1").arg(frame.people.size()));
    m_fps->setText(QString("FPS: %1").arg(frame.fps, 0, 'f', 1));
    m_latency->setText(QString("Latency: %1 ms")
                       .arg(frame.processingMs, 0, 'f', 1));
    m_frame->setText(QString("Frame: %1").arg(frame.frameNumber));

    m_table->setRowCount(frame.people.size());
    for (int row = 0; row < frame.people.size(); ++row)
    {
        const auto& person = frame.people[row];
        m_table->setItem(row, 0,
            new QTableWidgetItem(QString::number(person.id)));
        m_table->setItem(row, 1,
            new QTableWidgetItem(QString("%1 m").arg(person.distance, 0, 'f', 2)));
        m_table->setItem(row, 2,
            new QTableWidgetItem(QString("%1 m").arg(person.height, 0, 'f', 2)));
        m_table->setItem(row, 3,
            new QTableWidgetItem(QString("%1 m/s").arg(person.speed, 0, 'f', 2)));
        m_table->setItem(row, 4,
            new QTableWidgetItem(QString::number(person.points)));
        m_table->setItem(row, 5,
            new QTableWidgetItem(person.moving ? "MOVING" : "STATIC"));
    }
}

void MainWindow::loadParameters()
{
    const auto d = m_detector->detectorConfig();
    const auto t = m_detector->trackerConfig();
    m_floats["voxel_size"]->setValue(d.voxel_size);
    m_floats["ground_distance"]->setValue(d.ground_distance);
    m_floats["cluster_tolerance"]->setValue(d.cluster_tolerance);
    m_floats["min_height"]->setValue(d.min_height);
    m_floats["max_height"]->setValue(d.max_height);
    m_floats["min_width"]->setValue(d.min_width);
    m_floats["max_width"]->setValue(d.max_width);
    m_floats["min_depth"]->setValue(d.min_depth);
    m_floats["max_depth"]->setValue(d.max_depth);
    m_floats["min_range"]->setValue(d.min_range);
    m_floats["max_range"]->setValue(d.max_range);
    m_floats["max_horizontal_angle_deg"]->setValue(d.max_horizontal_angle_deg);
    m_floats["max_flatness"]->setValue(d.max_flatness);
    m_ints["ground_iterations"]->setValue(static_cast<int>(d.ground_iterations));
    m_ints["min_cluster_size"]->setValue(static_cast<int>(d.min_cluster_size));
    m_ints["max_cluster_size"]->setValue(static_cast<int>(d.max_cluster_size));
    m_ints["min_person_points"]->setValue(static_cast<int>(d.min_person_points));
    m_bools["enable_edge_rejection"]->setChecked(d.enable_edge_rejection);
    m_bools["enable_flat_object_rejection"]->setChecked(d.enable_flat_object_rejection);
    m_floats["max_association_distance"]->setValue(t.max_association_distance);
    m_floats["max_prediction_distance"]->setValue(t.max_prediction_distance);
    m_floats["min_movement"]->setValue(t.min_movement);
    m_floats["fallback_dt"]->setValue(t.fallback_dt);
    m_ints["max_missed_frames"]->setValue(t.max_missed_frames);
}

void MainWindow::applyParameters()
{
    auto d = m_detector->detectorConfig();
    auto t = m_detector->trackerConfig();
    d.voxel_size = static_cast<float>(m_floats["voxel_size"]->value());
    d.ground_distance = static_cast<float>(m_floats["ground_distance"]->value());
    d.cluster_tolerance = static_cast<float>(m_floats["cluster_tolerance"]->value());
    d.min_height = static_cast<float>(m_floats["min_height"]->value());
    d.max_height = static_cast<float>(m_floats["max_height"]->value());
    d.min_width = static_cast<float>(m_floats["min_width"]->value());
    d.max_width = static_cast<float>(m_floats["max_width"]->value());
    d.min_depth = static_cast<float>(m_floats["min_depth"]->value());
    d.max_depth = static_cast<float>(m_floats["max_depth"]->value());
    d.min_range = static_cast<float>(m_floats["min_range"]->value());
    d.max_range = static_cast<float>(m_floats["max_range"]->value());
    d.max_horizontal_angle_deg = static_cast<float>(m_floats["max_horizontal_angle_deg"]->value());
    d.max_flatness = static_cast<float>(m_floats["max_flatness"]->value());
    d.ground_iterations = m_ints["ground_iterations"]->value();
    d.min_cluster_size = m_ints["min_cluster_size"]->value();
    d.max_cluster_size = m_ints["max_cluster_size"]->value();
    d.min_person_points = m_ints["min_person_points"]->value();
    d.enable_edge_rejection = m_bools["enable_edge_rejection"]->isChecked();
    d.enable_flat_object_rejection = m_bools["enable_flat_object_rejection"]->isChecked();
    t.max_association_distance = static_cast<float>(m_floats["max_association_distance"]->value());
    t.max_prediction_distance = static_cast<float>(m_floats["max_prediction_distance"]->value());
    t.min_movement = static_cast<float>(m_floats["min_movement"]->value());
    t.fallback_dt = static_cast<float>(m_floats["fallback_dt"]->value());
    t.max_missed_frames = m_ints["max_missed_frames"]->value();
    if (d.min_range >= d.max_range ||
        d.min_height >= d.max_height ||
        d.min_width >= d.max_width ||
        d.min_depth >= d.max_depth ||
        d.min_cluster_size > d.max_cluster_size) {
        QMessageBox::warning(this, "Invalid parameters",
            "Minimum values must be smaller than maximum values, and minimum cluster size cannot exceed maximum.");
        return;
    }
    m_detector->setDetectorConfig(d);
    m_detector->setTrackerConfig(t);
}

void MainWindow::beginLog()
{
    endLog();
    const QString root = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)
                         + "/rslidar_results";
    QDir().mkpath(root);
    m_sessionDir = root + "/run_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
    if (!QDir().mkpath(m_sessionDir)) {
        QMessageBox::warning(this, "Logging", "Cannot create result directory: " + m_sessionDir);
        return;
    }
    m_pipelineCsv.setFileName(m_sessionDir + "/pipeline_debug.csv");
    if (m_pipelineCsv.open(QIODevice::WriteOnly | QIODevice::Text)) {
        m_pipelineStream.setDevice(&m_pipelineCsv);
        m_pipelineStream << "timestamp,frame,raw_points,roi_points,voxel_points,ground_removed_points,non_ground_points,clusters,rejected_points,rejected_height,rejected_width,rejected_depth,rejected_range,rejected_angle,rejected_flatness,accepted\n";
        m_pipelineStream.flush();
    }
    m_detectionCsv.setFileName(m_sessionDir + "/detections.csv");
    m_frameCsv.setFileName(m_sessionDir + "/frames.csv");
    if (!m_detectionCsv.open(QIODevice::WriteOnly | QIODevice::Text) ||
        !m_frameCsv.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Logging", "Cannot open CSV files in " + m_sessionDir);
        endLog();
        return;
    }
    m_detectionStream.setDevice(&m_detectionCsv);
    m_frameStream.setDevice(&m_frameCsv);
    m_detectionStream << "timestamp,elapsed_s,frame,person_id,x,y,z,distance,width,depth,height,points,speed,moving,fps,processing_ms\n";
    m_frameStream << "timestamp,elapsed_s,frame,people_count,fps,processing_ms\n";
    QJsonObject params;
    for (auto it = m_floats.cbegin(); it != m_floats.cend(); ++it) params.insert(it.key(), it.value()->value());
    for (auto it = m_ints.cbegin(); it != m_ints.cend(); ++it) params.insert(it.key(), it.value()->value());
    for (auto it = m_bools.cbegin(); it != m_bools.cend(); ++it) params.insert(it.key(), it.value()->isChecked());
    QFile settings(m_sessionDir + "/parameters.json");
    if (settings.open(QIODevice::WriteOnly)) settings.write(QJsonDocument(params).toJson());
    m_sessionTimer.start();
    m_logPreview->clear();
    m_logPath->setText("Saving to: " + m_sessionDir);
}

void MainWindow::endLog()
{
    if (m_pipelineCsv.isOpen()) {
        m_pipelineStream.flush();
        m_pipelineStream.setDevice(nullptr);
        m_pipelineCsv.close();
    }
    m_detectionStream.flush();
    m_frameStream.flush();
    m_detectionStream.setDevice(nullptr);
    m_frameStream.setDevice(nullptr);
    if (m_detectionCsv.isOpen()) m_detectionCsv.close();
    if (m_frameCsv.isOpen()) m_frameCsv.close();
}

void MainWindow::appendLog(const DetectionFrame& frame)
{
    if (!m_frameCsv.isOpen() || !m_detectionCsv.isOpen()) return;
    const QString stamp = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    const double elapsed = m_sessionTimer.elapsed() / 1000.0;
    m_frameStream << stamp << ',' << elapsed << ',' << frame.frameNumber << ','
                  << frame.people.size() << ',' << frame.fps << ',' << frame.processingMs << '\n';
    for (const auto& person : frame.people) {
        m_detectionStream << stamp << ',' << elapsed << ',' << frame.frameNumber << ','
            << person.id << ',' << person.x << ',' << person.y << ',' << person.z << ','
            << person.distance << ',' << person.width << ',' << person.depth << ','
            << person.height << ',' << person.points << ',' << person.speed << ','
            << (person.moving ? 1 : 0) << ',' << frame.fps << ',' << frame.processingMs << '\n';
    }
    m_frameStream.flush();
    m_detectionStream.flush();
    m_logPreview->appendPlainText(QString("Frame %1 | People %2 | FPS %3 | %4 ms")
        .arg(frame.frameNumber).arg(frame.people.size()).arg(frame.fps, 0, 'f', 1)
        .arg(frame.processingMs, 0, 'f', 1));
}
