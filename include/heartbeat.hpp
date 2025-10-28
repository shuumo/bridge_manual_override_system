/* Purpose:
 * Abstracts periodic polling (used by connection manager but modular).
*/

#pragma once
#include <QObject>
#include <QTimer>

class Heartbeat : public QObject {
    Q_OBJECT
public:
    explicit Heartbeat(QObject* parent = nullptr);
    void start(int intervalMs);
    void stop();

signals:
    void tick();

private:
    QTimer* m_timer;
};

