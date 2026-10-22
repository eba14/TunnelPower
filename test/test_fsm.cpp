// Native (PC) test of the Fsm state-transition logic (src/fsm.h) against a fake VFD pair --
// the one piece of core firmware logic that had never been automatically tested before this,
// only manually traced and exercised once on real hardware via physical pin touches.
// Build: g++ -std=c++17 -I src test/test_fsm.cpp -o test_fsm

#include "../src/fsm.h"
#include "fake_fsm_actions.h"
#include <cstdio>

int failures = 0;

void check(bool cond, const char *label) {
    if (!cond) {
        std::printf("FAIL: %s\n", label);
        failures++;
    } else {
        std::printf("ok:   %s\n", label);
    }
}

int main() {
    // IDLE -> PUMP_LEAD_WAIT on MCR HIGH, pump start succeeds.
    {
        Fsm fsm;
        FakeFsmActions actions;
        fsm.tick(true, 0, actions);
        check(fsm.state() == State::PUMP_LEAD_WAIT, "IDLE + MCR high + pump OK -> PUMP_LEAD_WAIT");
        check(actions.pumpStartCalls == 1 && actions.cutterStartCalls == 0,
              "only pump start attempted, cutter untouched");
    }

    // IDLE -> FAULT if the pump fails to start; cutter must never be attempted.
    {
        Fsm fsm;
        FakeFsmActions actions;
        actions.pumpStartSucceeds = false;
        fsm.tick(true, 0, actions);
        check(fsm.state() == State::FAULT, "IDLE + pump start fails -> FAULT");
        check(actions.cutterStartCalls == 0, "cutter never attempted when pump fails first");
    }

    // IDLE stays IDLE while MCR is low.
    {
        Fsm fsm;
        FakeFsmActions actions;
        fsm.tick(false, 0, actions);
        check(fsm.state() == State::IDLE, "IDLE + MCR low -> stays IDLE");
        check(actions.pumpStartCalls == 0, "no action taken while MCR low");
    }

    // PUMP_LEAD_WAIT: MCR drops before the lead time elapses -> abort, pump stopped, back to IDLE.
    {
        Fsm fsm;
        FakeFsmActions actions;
        fsm.tick(true, 0, actions);                       // -> PUMP_LEAD_WAIT at t=0
        fsm.tick(false, PUMP_LEAD_MS / 2, actions);        // MCR drops mid-wait
        check(fsm.state() == State::IDLE, "MCR drop during pump lead aborts back to IDLE");
        check(actions.pumpStopCalls == 1, "pump stop sent on abort");
        check(actions.cutterStartCalls == 0, "cutter never attempted on aborted start");
    }

    // PUMP_LEAD_WAIT: doesn't advance before PUMP_LEAD_MS elapses, even with MCR still high.
    {
        Fsm fsm;
        FakeFsmActions actions;
        fsm.tick(true, 0, actions);                        // -> PUMP_LEAD_WAIT at t=0
        fsm.tick(true, PUMP_LEAD_MS - 1, actions);          // 1ms short of the threshold
        check(fsm.state() == State::PUMP_LEAD_WAIT, "stays in PUMP_LEAD_WAIT 1ms before threshold");
        check(actions.cutterStartCalls == 0, "cutter not attempted before the lead time elapses");
    }

    // PUMP_LEAD_WAIT -> RUNNING once PUMP_LEAD_MS elapses and cutter start succeeds.
    {
        Fsm fsm;
        FakeFsmActions actions;
        fsm.tick(true, 0, actions);
        fsm.tick(true, PUMP_LEAD_MS, actions);
        check(fsm.state() == State::RUNNING, "PUMP_LEAD_WAIT -> RUNNING at exactly PUMP_LEAD_MS");
        check(actions.cutterStartCalls == 1, "cutter start attempted exactly once");
    }

    // PUMP_LEAD_WAIT -> FAULT if cutter fails to start; pump must be stopped too (don't leave
    // it running for nothing).
    {
        Fsm fsm;
        FakeFsmActions actions;
        actions.cutterStartSucceeds = false;
        fsm.tick(true, 0, actions);
        fsm.tick(true, PUMP_LEAD_MS, actions);
        check(fsm.state() == State::FAULT, "cutter start failure -> FAULT");
        check(actions.pumpStopCalls == 1, "pump stopped after cutter failed to start");
    }

    // RUNNING stays RUNNING while MCR holds; drops to CUTTER_SETTLE_WAIT when MCR releases.
    {
        Fsm fsm;
        FakeFsmActions actions;
        fsm.tick(true, 0, actions);
        fsm.tick(true, PUMP_LEAD_MS, actions);              // -> RUNNING
        fsm.tick(true, PUMP_LEAD_MS + 1000, actions);
        check(fsm.state() == State::RUNNING, "RUNNING holds while MCR stays high");
        fsm.tick(false, PUMP_LEAD_MS + 2000, actions);
        check(fsm.state() == State::CUTTER_SETTLE_WAIT, "MCR low while RUNNING -> CUTTER_SETTLE_WAIT");
        check(actions.cutterStopCalls == 1, "cutter stop sent exactly once");
    }

    // CUTTER_SETTLE_WAIT doesn't advance before CUTTER_STOP_SETTLE_MS, then stops the pump.
    {
        Fsm fsm;
        FakeFsmActions actions;
        fsm.tick(true, 0, actions);
        fsm.tick(true, PUMP_LEAD_MS, actions);
        fsm.tick(false, PUMP_LEAD_MS + 10, actions);        // -> CUTTER_SETTLE_WAIT at t=PUMP_LEAD_MS+10
        fsm.tick(false, PUMP_LEAD_MS + 10 + CUTTER_STOP_SETTLE_MS - 1, actions);
        check(fsm.state() == State::CUTTER_SETTLE_WAIT, "stays in CUTTER_SETTLE_WAIT 1ms before threshold");
        check(actions.pumpStopCalls == 0, "pump not stopped before settle time elapses");

        fsm.tick(false, PUMP_LEAD_MS + 10 + CUTTER_STOP_SETTLE_MS, actions);
        check(fsm.state() == State::IDLE, "CUTTER_SETTLE_WAIT -> IDLE once settle time elapses");
        check(actions.pumpStopCalls == 1, "pump stopped exactly once");
    }

    // FAULT only clears when MCR cycles back to low; stays FAULT while MCR remains high.
    {
        Fsm fsm;
        FakeFsmActions actions;
        actions.pumpStartSucceeds = false;
        fsm.tick(true, 0, actions);                         // -> FAULT
        fsm.tick(true, 1000, actions);
        check(fsm.state() == State::FAULT, "FAULT holds while MCR stays high");
        fsm.tick(false, 2000, actions);
        check(fsm.state() == State::IDLE, "FAULT clears to IDLE once MCR cycles low");
    }

    // Full happy-path integration: every action fires exactly once, in the right order.
    {
        Fsm fsm;
        FakeFsmActions actions;
        fsm.tick(true, 0, actions);                         // IDLE -> PUMP_LEAD_WAIT
        fsm.tick(true, PUMP_LEAD_MS, actions);               // -> RUNNING
        fsm.tick(false, PUMP_LEAD_MS + 1, actions);          // -> CUTTER_SETTLE_WAIT
        fsm.tick(false, PUMP_LEAD_MS + 1 + CUTTER_STOP_SETTLE_MS, actions);  // -> IDLE
        check(fsm.state() == State::IDLE, "full cycle returns to IDLE");
        check(actions.pumpStartCalls == 1 && actions.cutterStartCalls == 1 &&
              actions.cutterStopCalls == 1 && actions.pumpStopCalls == 1,
              "every action called exactly once across a full start/stop cycle");
    }

    // RUNNING + a fault appears mid-operation -> both VFDs stopped, straight to FAULT. This
    // is the blind spot that existed before anyFault() was added: a fault that occurs after
    // a successful start ACK, not during it, was previously invisible to the firmware.
    {
        Fsm fsm;
        FakeFsmActions actions;
        fsm.tick(true, 0, actions);
        fsm.tick(true, PUMP_LEAD_MS, actions);              // -> RUNNING
        check(fsm.state() == State::RUNNING, "reaches RUNNING before the fault appears");

        actions.faulted = true;
        fsm.tick(true, PUMP_LEAD_MS + 1000, actions);       // MCR still held, but now faulted
        check(fsm.state() == State::FAULT, "fault detected mid-RUNNING -> FAULT");
        check(actions.cutterStopCalls == 1 && actions.pumpStopCalls == 1,
              "both VFDs stopped when a mid-operation fault is detected");
    }

    // RUNNING with no fault present doesn't needlessly stop anything.
    {
        Fsm fsm;
        FakeFsmActions actions;
        fsm.tick(true, 0, actions);
        fsm.tick(true, PUMP_LEAD_MS, actions);              // -> RUNNING
        fsm.tick(true, PUMP_LEAD_MS + 1000, actions);
        check(fsm.state() == State::RUNNING, "stays RUNNING when anyFault() reports healthy");
        check(actions.anyFaultCalls >= 1, "fault status is actually being checked while RUNNING");
        check(actions.cutterStopCalls == 0 && actions.pumpStopCalls == 0,
              "nothing stopped when there's no fault");
    }

    std::printf("\n%s\n", failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED");
    return failures == 0 ? 0 : 1;
}
