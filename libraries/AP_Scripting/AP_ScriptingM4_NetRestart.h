#pragma once

/*
  Restart backoff for the M4 when it runs the network: every fault restarts the whole M4 after
  1 s, doubling to 30 s, retried forever; a session that ran HEALTHY_MS starts the next backoff
  from 1 s again. C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace ScriptingM4 {

class NetRestart {
public:
    static const uint32_t FIRST_DELAY_MS = 1000;
    static const uint32_t MAX_DELAY_MS = 30000;
    static const uint32_t HEALTHY_MS = 60000;

    void session_started(uint32_t now_ms)
    {
        _session_start_ms = now_ms;
        _running = true;
    }
    // the delay before the restart grant
    uint32_t fault(uint32_t now_ms)
    {
        if (_running && now_ms - _session_start_ms >= HEALTHY_MS) {
            _delay_ms = 0;
        }
        _running = false;
        _delay_ms = _delay_ms == 0 ? FIRST_DELAY_MS
                                   : (_delay_ms >= MAX_DELAY_MS / 2 ? MAX_DELAY_MS : _delay_ms * 2);
        _due_ms = now_ms + _delay_ms;
        _count++;
        return _delay_ms;
    }
    bool due(uint32_t now_ms) const { return int32_t(now_ms - _due_ms) >= 0; }
    uint32_t count() const { return _count; }

private:
    uint32_t _session_start_ms = 0;
    uint32_t _due_ms = 0;
    uint32_t _delay_ms = 0;
    uint32_t _count = 0;
    bool _running = false;
};

}  // namespace ScriptingM4
