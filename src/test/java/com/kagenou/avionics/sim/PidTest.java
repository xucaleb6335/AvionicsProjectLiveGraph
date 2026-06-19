package com.kagenou.avionics.sim;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Unit tests for the single-axis PID with anti-windup and output saturation. */
class PidTest {

    @Test
    void proportionalOnlyReturnsGainTimesError() {
        Pid pid = new Pid(2f, 0f, 0f, 100f, 100f, 0.02f);
        // First call: no integral contribution (ki=0), no derivative (no history yet).
        assertEquals(10f, pid.update(5f, 0f, 0.01f), 1e-4f); // 2 * (5 - 0)
    }

    @Test
    void outputSaturatesAtOutMax() {
        Pid pid = new Pid(2f, 0f, 0f, 4f, 100f, 0.02f);
        assertEquals(4f, pid.update(5f, 0f, 0.01f), 1e-4f);  // raw 10 clamped to 4
        Pid neg = new Pid(2f, 0f, 0f, 4f, 100f, 0.02f);
        assertEquals(-4f, neg.update(-5f, 0f, 0.01f), 1e-4f);
    }

    @Test
    void integralAccumulatesThenResetClearsIt() {
        Pid pid = new Pid(0f, 1f, 0f, 100f, 100f, 0.02f); // integral-only
        assertEquals(0.1f, pid.update(1f, 0f, 0.1f), 1e-4f); // integral = 0.1
        assertEquals(0.2f, pid.update(1f, 0f, 0.1f), 1e-4f); // integral = 0.2
        pid.reset();
        assertEquals(0.1f, pid.update(1f, 0f, 0.1f), 1e-4f); // back to a single step
    }

    @Test
    void integralTermIsBoundedByAntiWindup() {
        // iTermMax bounds |ki * integral|; drive a large sustained error.
        Pid pid = new Pid(0f, 2f, 0f, 100f, 0.5f, 0.02f);
        float out = 0f;
        for (int i = 0; i < 1000; i++) {
            out = pid.update(10f, 0f, 0.01f);
        }
        assertTrue(Math.abs(out) <= 0.5f + 1e-4f, "I-term should be clamped to iTermMax, was " + out);
    }
}
