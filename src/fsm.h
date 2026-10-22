#pragma once
#include "config.h"

enum class State { IDLE, PUMP_LEAD_WAIT, RUNNING, CUTTER_SETTLE_WAIT, FAULT };

// Actions the FSM requests; the concrete implementation decides how (real Modbus commands on
// the ESP32, canned results in tests). Kept free of Arduino/hardware calls so Fsm is testable.
class FsmActions {
public:
    virtual ~FsmActions() = default;
    virtual bool pumpStart() = 0;    // true on success
    virtual bool cutterStart() = 0;  // true on success
    virtual void cutterStop() = 0;   // unconditional -- see attemptCutterStop() in main.cpp
    virtual void pumpStop() = 0;     // unconditional
    virtual bool anyFault() = 0;     // true if either VFD reports a fault right now
};

class Fsm {
public:
    State state() const { return currentState; }

    // Advances the FSM by one tick. mcrActive = current MCR reading, now = current millis().
    void tick(bool mcrActive, unsigned long now, FsmActions &actions) {
        switch (currentState) {
            case State::IDLE:
                if (mcrActive) {
                    enterState(actions.pumpStart() ? State::PUMP_LEAD_WAIT : State::FAULT, now);
                }
                break;

            case State::PUMP_LEAD_WAIT:
                if (!mcrActive) {
                    actions.pumpStop();
                    enterState(State::IDLE, now);
                } else if (now - stateEnteredAt >= PUMP_LEAD_MS) {
                    if (actions.cutterStart()) {
                        enterState(State::RUNNING, now);
                    } else {
                        actions.pumpStop();
                        enterState(State::FAULT, now);
                    }
                }
                break;

            case State::RUNNING:
                if (!mcrActive) {
                    actions.cutterStop();
                    enterState(State::CUTTER_SETTLE_WAIT, now);
                } else if (actions.anyFault()) {
                    // A VFD faulted mid-operation, not at start -- the only window the
                    // original command ACKs can't see. Stop both and surface it as FAULT.
                    actions.cutterStop();
                    actions.pumpStop();
                    enterState(State::FAULT, now);
                }
                break;

            case State::CUTTER_SETTLE_WAIT:
                if (now - stateEnteredAt >= CUTTER_STOP_SETTLE_MS) {
                    actions.pumpStop();
                    enterState(State::IDLE, now);
                }
                break;

            case State::FAULT:
                if (!mcrActive) {
                    enterState(State::IDLE, now);
                }
                break;
        }
    }

private:
    State currentState = State::IDLE;
    unsigned long stateEnteredAt = 0;

    void enterState(State s, unsigned long now) {
        currentState = s;
        stateEnteredAt = now;
    }
};
