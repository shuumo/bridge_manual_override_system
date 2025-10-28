The heartbeat (in ConnectionManager) sends "PING" to the ESP32 every second.

the ESP32 replies with:
"OK STATE=<OPEN|CLOSED|OPENING|...> MODE=<AUTO|OVERRIDE>".

the host parses that reply, updates BridgeStatus, and emits a statusUpdated() signal.

gui reacts instantly label text and button color updates.

override button toggles mode and sends a "MODE=OVERRIDE" or "MODE=AUTO" command to the ESP.

when override mode is active, the state buttons are enabled and send e.g. "SET_STATE OPENING".
