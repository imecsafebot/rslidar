#pragma once

#include "detectoradapter.h"
#include "pointcloudwidget.h"

#include <QMainWindow>
#include <QMap>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QElapsedTimer>

class QLabel;
class QPushButton;
class QTableWidget;
class QDoubleSpinBox;
class QSpinBox;
class QCheckBox;
class QTabWidget;
class QPlainTextEdit;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void startDetection();
    void stopDetection();
    void resetView();
    void updateFrame(const DetectionFrame& frame);
    void updateStatus(const QString& status);
    void showError(const QString& message);

private:
    void buildUi();
    void setRunningUi(bool running);
    void applyParameters();
    void loadParameters();
    void beginLog();
    void endLog();
    void appendLog(const DetectionFrame& frame);

    QFile m_detectionCsv;
    QFile m_frameCsv;
    QFile m_pipelineCsv;
    QTextStream m_pipelineStream;
    QLabel* m_pipelineSummary = nullptr;
    QTextStream m_detectionStream;
    QTextStream m_frameStream;
    QElapsedTimer m_sessionTimer;
    QString m_sessionDir;
    QPlainTextEdit* m_logPreview = nullptr;
    QLabel* m_logPath = nullptr;
    PointCloudWidget* m_view = nullptr;
    DetectorAdapter* m_detector = nullptr;
    QLabel* m_connection = nullptr;
    QLabel* m_runState = nullptr;
    QLabel* m_people = nullptr;
    QLabel* m_fps = nullptr;
    QLabel* m_latency = nullptr;
    QLabel* m_frame = nullptr;
    QTableWidget* m_table = nullptr;
    QPushButton* m_start = nullptr;
    QPushButton* m_stop = nullptr;
    QPushButton* m_reset = nullptr;
    QPushButton* m_apply = nullptr;
    QMap<QString, QDoubleSpinBox*> m_floats;
    QMap<QString, QSpinBox*> m_ints;
    QMap<QString, QCheckBox*> m_bools;
};
