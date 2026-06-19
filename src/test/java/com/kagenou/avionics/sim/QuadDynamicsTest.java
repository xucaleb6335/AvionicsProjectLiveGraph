package com.kagenou.avionics.sim;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Unit tests for the quad rigid-body / vertical dynamics. */
class QuadDynamicsTest {

    private static float[] perMotor(float total) {
        float each = total / 4f;
        return new float[]{each, each, each, each};
    }

    @Test
    void hoverThrustBalancesWeight() {
        QuadDynamics q = new QuadDynamics();
        assertEquals(q.mass * q.g, q.hoverThrust(), 1e-4f);
    }

    @Test
    void exactHoverThrustHoldsAltitude() {
        QuadDynamics q = new QuadDynamics();
        q.reset();
        float[] cmd = perMotor(q.hoverThrust());
        for (int i = 0; i < 2000; i++) { // 4 s at 500 Hz
            q.step(0.002f, cmd);
        }
        // Motor lag causes a tiny initial sag; it should settle close to the start height.
        assertTrue(Math.abs(q.altitude()) < 0.2f, "altitude drifted to " + q.altitude());
    }

    @Test
    void excessThrustClimbs() {
        QuadDynamics q = new QuadDynamics();
        q.reset();
        float[] cmd = perMotor(q.hoverThrust() * 1.5f);
        for (int i = 0; i < 1000; i++) { // 2 s
            q.step(0.002f, cmd);
        }
        assertTrue(q.altitude() > 0.5f, "expected climb, altitude = " + q.altitude());
        assertTrue(q.verticalSpeed() > 0f, "expected positive vertical speed");
    }

    @Test
    void rateImpulseSpinsThenDampsTowardRest() {
        QuadDynamics q = new QuadDynamics();
        q.reset();
        q.applyRateImpulse(3f, 0f, 0f);
        assertTrue(q.rateX() > 0f, "impulse should produce a positive roll rate");

        float[] hover = perMotor(q.hoverThrust());
        float peak = q.rateX();
        for (int i = 0; i < 3000; i++) { // 6 s of aerodynamic damping, no control torque
            q.step(0.002f, hover);
        }
        assertTrue(q.rateX() < peak, "aerodynamic damping should reduce the roll rate");
    }
}
