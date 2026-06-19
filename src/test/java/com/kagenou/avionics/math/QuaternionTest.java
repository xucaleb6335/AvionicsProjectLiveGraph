package com.kagenou.avionics.math;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/** Unit tests for the quaternion attitude math. */
class QuaternionTest {

    private static final float D2R = (float) (Math.PI / 180.0);

    @Test
    void identityHasZeroEulerAngles() {
        float[] e = new Quaternion().identity().toEulerDeg();
        assertEquals(0f, e[0], 1e-4f);
        assertEquals(0f, e[1], 1e-4f);
        assertEquals(0f, e[2], 1e-4f);
    }

    @Test
    void normalizeProducesUnitQuaternion() {
        Quaternion q = new Quaternion(2f, 0f, 0f, 0f).normalize();
        float mag = (float) Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        assertEquals(1f, mag, 1e-5f);
    }

    @Test
    void eulerRoundTripRecoversModerateAngles() {
        // Use moderate angles well clear of the +/-90-deg pitch singularity.
        Quaternion q = Quaternion.fromEuler(10f * D2R, -20f * D2R, 30f * D2R);
        float[] e = q.toEulerDeg();
        assertEquals(10f, e[0], 0.5f);
        assertEquals(-20f, e[1], 0.5f);
        assertEquals(30f, e[2], 0.5f);
    }

    @Test
    void conjugateNegatesVectorPart() {
        // conjugate() mutates in place and returns this, so capture the originals first.
        float w = 0.5f, x = 0.1f, y = -0.2f, z = 0.3f;
        Quaternion c = new Quaternion(w, x, y, z).conjugate();
        assertEquals(w, c.w, 1e-6f);
        assertEquals(-x, c.x, 1e-6f);
        assertEquals(-y, c.y, 1e-6f);
        assertEquals(-z, c.z, 1e-6f);
    }

    @Test
    void multiplyingByIdentityIsANoOp() {
        Quaternion q = Quaternion.fromEuler(15f * D2R, 25f * D2R, -35f * D2R);
        Quaternion out = new Quaternion();
        Quaternion.mul(q, new Quaternion().identity(), out);
        assertEquals(q.w, out.w, 1e-5f);
        assertEquals(q.x, out.x, 1e-5f);
        assertEquals(q.y, out.y, 1e-5f);
        assertEquals(q.z, out.z, 1e-5f);
    }
}
