package com.kagenou.avionics.math;

/**
 * Minimal quaternion used for IMU attitude.
 *
 * <p>Storage order is (w, x, y, z). Mutable so the render loop can reuse a small
 * number of instances per frame instead of allocating; the IMU streams faster
 * than we draw and we do not want GC churn in the hot loop.
 */
public final class Quaternion {
    public float w, x, y, z;

    public Quaternion() {
        this(1, 0, 0, 0); // identity
    }

    public Quaternion(float w, float x, float y, float z) {
        set(w, x, y, z);
    }

    public Quaternion set(float w, float x, float y, float z) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    public Quaternion set(Quaternion q) {
        return set(q.w, q.x, q.y, q.z);
    }

    public Quaternion identity() {
        return set(1, 0, 0, 0);
    }

    /**
     * Normalizes in place. A zero/degenerate quaternion (e.g. before the first
     * sample arrives, or a garbage line that slipped through) collapses to
     * identity rather than producing NaNs in the rotation matrix.
     */
    public Quaternion normalize() {
        float n = (float) Math.sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-8f) {
            return identity();
        }
        float inv = 1f / n;
        w *= inv;
        x *= inv;
        y *= inv;
        z *= inv;
        return this;
    }

    /**
     * Writes this rotation into {@code m} as a 4x4 column-major matrix, ready to
     * hand straight to {@code glMultMatrixf}. Assumes a unit quaternion.
     */
    public void toMatrixColumnMajor(float[] m) {
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;

        // column 0
        m[0] = 1 - 2 * (yy + zz);
        m[1] = 2 * (xy + wz);
        m[2] = 2 * (xz - wy);
        m[3] = 0;
        // column 1
        m[4] = 2 * (xy - wz);
        m[5] = 1 - 2 * (xx + zz);
        m[6] = 2 * (yz + wx);
        m[7] = 0;
        // column 2
        m[8] = 2 * (xz + wy);
        m[9] = 2 * (yz - wx);
        m[10] = 1 - 2 * (xx + yy);
        m[11] = 0;
        // column 3
        m[12] = 0;
        m[13] = 0;
        m[14] = 0;
        m[15] = 1;
    }

    /**
     * Converts to aerospace Euler angles in degrees as {@code [roll, pitch, yaw]}
     * (X, Y, Z). Handy for the HUD/diagnostics and for sanity-logging the feed.
     */
    public float[] toEulerDeg() {
        // roll (x-axis)
        float sinrCosp = 2 * (w * x + y * z);
        float cosrCosp = 1 - 2 * (x * x + y * y);
        float roll = (float) Math.atan2(sinrCosp, cosrCosp);

        // pitch (y-axis), clamped to avoid NaN from asin at the poles
        float sinp = 2 * (w * y - z * x);
        float pitch = Math.abs(sinp) >= 1
                ? (float) Math.copySign(Math.PI / 2, sinp)
                : (float) Math.asin(sinp);

        // yaw (z-axis)
        float sinyCosp = 2 * (w * z + x * y);
        float cosyCosp = 1 - 2 * (y * y + z * z);
        float yaw = (float) Math.atan2(sinyCosp, cosyCosp);

        float r2d = (float) (180.0 / Math.PI);
        return new float[]{roll * r2d, pitch * r2d, yaw * r2d};
    }

    /** Conjugate in place; equals the inverse for a unit quaternion. */
    public Quaternion conjugate() {
        x = -x;
        y = -y;
        z = -z;
        return this;
    }

    /** Hamilton product {@code out = a ⊗ b}. Safe if {@code out} aliases {@code a} or {@code b}. */
    public static Quaternion mul(Quaternion a, Quaternion b, Quaternion out) {
        float w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        float x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        float y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
        float z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
        return out.set(w, x, y, z);
    }

    /** Builds a unit quaternion from aerospace Euler angles (radians, ZYX order). */
    public static Quaternion fromEuler(float roll, float pitch, float yaw) {
        float cr = (float) Math.cos(roll * 0.5f), sr = (float) Math.sin(roll * 0.5f);
        float cp = (float) Math.cos(pitch * 0.5f), sp = (float) Math.sin(pitch * 0.5f);
        float cy = (float) Math.cos(yaw * 0.5f), sy = (float) Math.sin(yaw * 0.5f);
        return new Quaternion(
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy);
    }

    /**
     * Spherically interpolates this quaternion a fraction {@code t} toward
     * {@code target}, in place. Used to smooth render-rate jitter from the feed;
     * {@code t == 1} means "snap to target" (no smoothing).
     */
    public Quaternion slerpTo(Quaternion target, float t) {
        if (t >= 1f) {
            return set(target).normalize();
        }
        float dot = w * target.w + x * target.x + y * target.y + z * target.z;

        // Take the shorter arc.
        float tw = target.w, tx = target.x, ty = target.y, tz = target.z;
        if (dot < 0f) {
            dot = -dot;
            tw = -tw; tx = -tx; ty = -ty; tz = -tz;
        }

        if (dot > 0.9995f) {
            // Nearly parallel: linear interpolate then renormalize, avoids div-by-~0.
            w += t * (tw - w);
            x += t * (tx - x);
            y += t * (ty - y);
            z += t * (tz - z);
            return normalize();
        }

        float theta0 = (float) Math.acos(dot);
        float theta = theta0 * t;
        float sinTheta0 = (float) Math.sin(theta0);
        float s0 = (float) Math.cos(theta) - dot * (float) Math.sin(theta) / sinTheta0;
        float s1 = (float) Math.sin(theta) / sinTheta0;

        return set(
                s0 * w + s1 * tw,
                s0 * x + s1 * tx,
                s0 * y + s1 * ty,
                s0 * z + s1 * tz).normalize();
    }
}
