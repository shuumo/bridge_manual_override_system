#include "heartbeat.hpp"
#include <QDebug>

Heartbeat::Heartbeat(QObject* parent)
    : QObject(parent),
      m_timer(new QTimer(this))
{
    connect(m_timer, &QTimer::timeout, this, &Heartbeat::tick);
}

void Heartbeat::start(int intervalMs) {
    if (!m_timer->isActive()) {
        m_timer->start(intervalMs);
        qDebug() << "Heartbeat started at" << intervalMs << "ms interval.";
    }
}

void Heartbeat::stop() {
    if (m_timer->isActive()) {
        m_timer->stop();
        qDebug() << "Heartbeat stopped.";
    }
}

