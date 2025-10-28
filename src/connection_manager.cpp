#include "connection_manager.hpp"
#include <QNetworkDatagram>
#include <QHostAddress>
#include <QStringList>
#include <QDebug>

ConnectionManager::ConnectionManager(QObject* parent)
    : QObject(parent),
      m_socket(new QUdpSocket(this)),
      m_heartbeatTimer(new QTimer(this)),
      m_espPort(0),
      m_missedReplies(0)
{
    m_status = {BridgeState::INACTIVE, ControlMode::AUTO, false};

    connect(m_socket, &QUdpSocket::readyRead,
            this, &ConnectionManager::processPendingDatagrams);

    connect(m_heartbeatTimer, &QTimer::timeout,
            this, &ConnectionManager::sendHeartbeat);

    m_heartbeatTimer->setInterval(1000);
}

void ConnectionManager::start(const QString& espAddress, quint16 espPort) {
    m_espAddress = espAddress;
    m_espPort = espPort;

    if (!m_socket->bind(QHostAddress::AnyIPv4, 0, QUdpSocket::ShareAddress)) {
        qWarning() << "Failed to bind UDP socket.";
        return;
    }

    m_heartbeatTimer->start();
    qInfo() << "ConnectionManager started heartbeat to" << espAddress << ":" << espPort;
}

void ConnectionManager::stop() {
    m_heartbeatTimer->stop();
    m_socket->close();
    m_status.connected = false;
    emit connectionLost();
}

void ConnectionManager::sendHeartbeat() {
    QByteArray ping("PING");
    m_socket->writeDatagram(ping, QHostAddress(m_espAddress), m_espPort);

    m_missedReplies++;
    if (m_missedReplies >= 3) {
        if (m_status.connected) {
            m_status.connected = false;
            emit connectionLost();
            qWarning() << "Heartbeat lost.";
        }
    }
}

void ConnectionManager::processPendingDatagrams() {
    while (m_socket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = m_socket->receiveDatagram();
        QString msg = QString::fromUtf8(datagram.data()).trimmed();
        qDebug() << "Received:" << msg;

        if (msg.startsWith("OK")) {
            // Reset missed-reply counter
            m_missedReplies = 0;
            m_status.connected = true;
            QStringList parts = msg.split(' ', Qt::SkipEmptyParts);
            for (const QString& part : parts) {
                if (part.startsWith("STATE=")) {
                    QString state = part.section('=', 1, 1);
                    if (state == "IDLE") m_status.state = BridgeState::IDLE;
                    else if (state == "OPENING") m_status.state = BridgeState::OPENING;
                    else if (state == "OPEN") m_status.state = BridgeState::OPEN;
                    else if (state == "CLOSING") m_status.state = BridgeState::CLOSING;
                    else if (state == "ERROR") m_status.state = BridgeState::ERROR;
                    else m_status.state = BridgeState::INACTIVE;
                } else if (part.startsWith("MODE=")) {
                    QString mode = part.section('=', 1, 1);
                    if (mode == "AUTO") m_status.mode = ControlMode::AUTO;
                    else if (mode == "OVERRIDE") m_status.mode = ControlMode::OVERRIDE;
                }
            }

            emit statusUpdated(m_status);
        }
    }
}

void ConnectionManager::sendCommand(const QString& cmd) {
    if (m_espAddress.isEmpty()) return;
    QByteArray data = cmd.toUtf8();
    qint64 bytes = m_socket->writeDatagram(data, QHostAddress(m_espAddress), m_espPort);
    if (bytes == -1)
        qWarning() << "Failed to send command:" << cmd;
}

