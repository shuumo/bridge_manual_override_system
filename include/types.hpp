/* PURPOSE:
 * Define shared enums and structs, what both the state machine and connection manager use.
*/

#pragma once
#include <QString>

enum class BridgeState {
    INACTIVE,
    IDLE,
    OPENING,
    OPEN,
    CLOSING,
    ERROR
};

enum class ControlMode {
    AUTO,
    OVERRIDE
};

struct BridgeStatus {
    BridgeState state;
    ControlMode mode;
    bool connected;
};

