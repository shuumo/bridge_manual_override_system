/* PURPOSE
 * Handles all UDP communications to/from the ESP32
*/

#pragma once
#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include "types.hpp"

class ConnectionManager : public QObject {
    Q_OBJECT
public:
    explicit ConnectionManager(QObject* parent = nullptr);
    void start(const QString& espAddress, quint16 espPort);
    void stop();
    void sendCommand(const QString& cmd);
    BridgeStatus getStatus() const { return m_status; }

signals:
    void statusUpdated(const BridgeStatus& status);
    void connectionLost();

private slots:
    void sendHeartbeat();
    void processPendingDatagrams();

private:
    QUdpSocket* m_socket;
    QTimer* m_heartbeatTimer;
    QString m_espAddress;
    quint16 m_espPort;

    BridgeStatus m_status;
    int m_missedReplies = 0;
};

