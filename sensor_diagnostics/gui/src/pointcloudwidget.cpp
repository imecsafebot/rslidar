#include "pointcloudwidget.h"

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

PointCloudWidget::PointCloudWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(760, 560);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
}

void PointCloudWidget::setFrame(const DetectionFrame& frame)
{
    m_frame = frame;
    update();
}

void PointCloudWidget::resetView()
{
    m_yaw = 0.0f;
    m_pitch = 20.0f;
    m_zoom = 1.0f;
    update();
}

QPointF PointCloudWidget::project(float x, float y, float z) const
{
    constexpr float pi = 3.14159265358979323846f;
    const float yaw = m_yaw * pi / 180.0f;
    const float pitch = m_pitch * pi / 180.0f;

    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);
    const float x1 = cy * x - sy * y;
    const float y1 = sy * x + cy * y;

    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float y2 = cp * y1 - sp * z;
    const float z2 = sp * y1 + cp * z;

    Q_UNUSED(y2);

    const float scale = std::min(width(), height()) * 0.09f * m_zoom;
    return QPointF(width() * 0.5 + x1 * scale,
                   height() * 0.74 - z2 * scale);
}

void PointCloudWidget::drawGrid(QPainter& painter)
{
    painter.setPen(QPen(QColor(54, 62, 73), 1));

    for (int i = -10; i <= 10; ++i)
    {
        painter.drawLine(project(static_cast<float>(i), -10.0f, 0.0f),
                         project(static_cast<float>(i), 10.0f, 0.0f));
        painter.drawLine(project(-10.0f, static_cast<float>(i), 0.0f),
                         project(10.0f, static_cast<float>(i), 0.0f));
    }
}

void PointCloudWidget::drawPersonBox(QPainter& painter, const GuiPerson& person)
{
    const float hx = person.width * 0.5f;
    const float hy = person.depth * 0.5f;
    const float hz = person.height * 0.5f;
    const float x = person.x;
    const float y = person.y;
    const float z = person.z;

    const QPointF c[8] = {
        project(x-hx,y-hy,z-hz), project(x-hx,y-hy,z+hz),
        project(x-hx,y+hy,z-hz), project(x-hx,y+hy,z+hz),
        project(x+hx,y-hy,z-hz), project(x+hx,y-hy,z+hz),
        project(x+hx,y+hy,z-hz), project(x+hx,y+hy,z+hz)
    };

    const int edges[][2] = {
        {0,1},{0,2},{0,4},{1,3},{1,5},{2,3},
        {2,6},{4,5},{4,6},{3,7},{5,7},{6,7}
    };

    painter.setPen(QPen(person.moving ? QColor(255,85,85)
                                      : QColor(80,235,125), 3));
    for (const auto& edge : edges)
        painter.drawLine(c[edge[0]], c[edge[1]]);

    const QPointF label = project(x, y, z + hz + 0.15f);
    painter.setPen(Qt::white);
    painter.setFont(QFont("Sans", 10, QFont::Bold));
    painter.drawText(label + QPointF(7, -5),
                     QString("ID %1  %2 m").arg(person.id)
                         .arg(person.distance, 0, 'f', 1));
}

void PointCloudWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.fillRect(rect(), QColor(13, 17, 22));

    drawGrid(painter);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(185, 195, 205));

    // Rendering only: detection still receives the complete processed cloud.
    const int stride = std::max(1, m_frame.points.size() / 14000);
    for (int i = 0; i < m_frame.points.size(); i += stride)
    {
        const auto& p = m_frame.points[i];
        painter.drawEllipse(project(p.x, p.y, p.z), 1.4, 1.4);
    }

    for (const auto& person : m_frame.people)
        drawPersonBox(painter, person);

    painter.setPen(QColor(160, 170, 182));
    painter.setFont(QFont("Sans", 9));
    painter.drawText(12, height() - 12,
                     "Drag: rotate    Wheel: zoom    Reset View: restore camera");
}

void PointCloudWidget::mousePressEvent(QMouseEvent* event)
{
    m_lastMouse = event->pos();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent* event)
{
    const QPoint delta = event->pos() - m_lastMouse;
    m_lastMouse = event->pos();

    if (event->buttons() & Qt::LeftButton)
    {
        m_yaw += delta.x() * 0.6f;
        m_pitch = std::clamp(m_pitch - delta.y() * 0.5f, -85.0f, 85.0f);
        update();
    }
}

void PointCloudWidget::wheelEvent(QWheelEvent* event)
{
    m_zoom *= event->angleDelta().y() > 0 ? 1.12f : 0.89f;
    m_zoom = std::clamp(m_zoom, 0.25f, 5.0f);
    update();
}
