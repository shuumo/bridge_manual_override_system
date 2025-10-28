/* PURPOSE:
 * Manage bridge state transitions and keep the logic consistent (GUI + communication sync).
*/

#pragma once
#include <QObject>
#include "types.hpp"

class StateMachine : public QObject {
    Q_OBJECT
public:
    explicit StateMachine(QObject* parent = nullptr);
    BridgeState currentState() const { return m_state; }
    ControlMode currentMode() const { return m_mode; }

    void setState(BridgeState newState);
    void setMode(ControlMode newMode);

signals:
    void stateChanged(BridgeState newState);
    void modeChanged(ControlMode newMode);

private:
    BridgeState m_state;
    ControlMode m_mode;
};

