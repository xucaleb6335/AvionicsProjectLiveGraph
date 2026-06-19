package com.kagenou.avionics.sim;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Closed-loop tests for the cascaded PID flight controller driving the quad model.
 * These exercise the same physics the visualizer renders, headlessly.
 */
class SimulatorTest {

    /** Steps the simulator forward by {@code seconds} in fixed display-sized ticks. */
    private static void run(Simulator sim, double seconds) {
        for (double t = 0; t < seconds; t += 0.02) {
            sim.step(0.02);
        }
    }

    @Test
    void tunedControllerTracksARollStep() {
        Simulator sim = new Simulator(); // defaults to the Tuned PID preset
        sim.commandStep(20f);
        run(sim, 5.0);
        float roll = sim.attitude().toEulerDeg()[0];
        assertEquals(20f, roll, 3f, "roll should settle near the 20-deg setpoint");
    }

    @Test
    void controllerRecoversFromADisturbance() {
        Simulator sim = new Simulator();
        sim.levelSetpoint();
        sim.disturb();          // injects a rate impulse
        run(sim, 6.0);
        float[] e = sim.attitude().toEulerDeg();
        assertTrue(Math.abs(e[0]) < 3f, "roll did not recover: " + e[0]);
        assertTrue(Math.abs(e[1]) < 3f, "pitch did not recover: " + e[1]);
    }

    @Test
    void altitudeHoldReachesSetpoint() {
        Simulator sim = new Simulator();
        run(sim, 8.0);
        assertEquals(sim.altSetpoint(), sim.altitude(), 0.2f, "altitude hold should reach the setpoint");
    }

    @Test
    void openLoopPresetHasNoFeedbackGains() {
        Simulator sim = new Simulator();
        int openLoop = indexOf(sim.presetNames(), "Open Loop");
        sim.applyPreset(openLoop);
        assertEquals(openLoop, sim.currentPreset());
        for (int i = 0; i < sim.gainCount(); i++) {
            assertEquals(0f, sim.gainValue(i), 1e-6f, "Open Loop gain " + i + " should be zero");
        }
    }

    @Test
    void setGainClampsAndMarksConfigCustom() {
        Simulator sim = new Simulator();
        sim.applyPreset(4); // a known preset
        sim.setGain(0, 1e6f); // way above max
        assertEquals(sim.gainMax(0), sim.gainValue(0), 1e-4f, "gain should clamp to its max");
        assertEquals(-1, sim.currentPreset(), "manual gain change should clear the active preset");
    }

    private static int indexOf(String[] arr, String name) {
        for (int i = 0; i < arr.length; i++) {
            if (arr[i].equals(name)) {
                return i;
            }
        }
        throw new IllegalArgumentException("no preset named " + name);
    }
}
