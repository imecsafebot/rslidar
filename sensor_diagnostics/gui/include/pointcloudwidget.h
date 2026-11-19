#pragma once

#include "detectoradapter.h"

#include <QPoint>
#include <QWidget>

class QPainter;
class QMouseEvent;
class QWheelEvent;

class PointCloudWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PointCloudWidget(QWidget* parent = nullptr);

    void setFrame(const DetectionFrame& frame);
    void resetView();

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;

private:
    QPointF project(float x, float y, float z) const;
    void drawGrid(QPainter& painter);
    void drawPersonBox(QPainter& painter, const GuiPerson& person);

    DetectionFrame m_frame;
    QPoint m_lastMouse;
    float m_yaw = 0.0f;
    float m_pitch = 20.0f;
    float m_zoom = 1.0f;
};
