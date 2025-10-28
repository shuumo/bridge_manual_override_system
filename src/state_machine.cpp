#include "state_machine.hpp"
#include <QDebug>

StateMachine::StateMachine(QObject* parent)
    : QObject(parent),
      m_state(BridgeState::INACTIVE),
      m_mode(ControlMode::AUTO)
{}

void StateMachine::setState(BridgeState newState) {
    if (m_state != newState) {
        m_state = newState;
        emit stateChanged(m_state);
        qDebug() << "StateMachine -> new state:" << static_cast<int>(m_state);
    }
}

void StateMachine::setMode(ControlMode newMode) {
    if (m_mode != newMode) {
        m_mode = newMode;
        emit modeChanged(m_mode);
        qDebug() << "StateMachine -> new mode:" << static_cast<int>(m_mode);
    }
}

