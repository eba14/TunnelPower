#pragma once
#include "../src/fsm.h"

// A fake VFD pair for testing Fsm::tick() without any hardware. Configure which actions
// should fail, then inspect the call log to see exactly what the FSM asked for and when.
class FakeFsmActions : public FsmActions {
public:
    bool pumpStartSucceeds = true;
    bool cutterStartSucceeds = true;
    bool faulted = false;

    int pumpStartCalls = 0;
    int cutterStartCalls = 0;
    int cutterStopCalls = 0;
    int pumpStopCalls = 0;
    int anyFaultCalls = 0;

    bool pumpStart() override {
        pumpStartCalls++;
        return pumpStartSucceeds;
    }

    bool cutterStart() override {
        cutterStartCalls++;
        return cutterStartSucceeds;
    }

    void cutterStop() override { cutterStopCalls++; }
    void pumpStop() override { pumpStopCalls++; }

    bool anyFault() override {
        anyFaultCalls++;
        return faulted;
    }
};
