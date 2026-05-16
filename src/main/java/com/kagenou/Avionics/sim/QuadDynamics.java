package com.kagenou.Avionics.sim;

import com.kagenou.Avionics.math.Quaternion;

/**
 * Rigid-body rotational dynamics of an X-configuration quadcopter.
 *
 * <p>Body frame: x forward, y right, z up. State is the attitude quaternion and
 * the body angular rates. Four motors produce upward thrust; their layout gives
 * the standard mixing:
 * <pre>
 *   tauX (roll)  = d * ( f1 + f2 - f3 - f4)
 *   tauY (pitch) = d * (-f1 + f2 + f3 - f4)
 *   tauZ (yaw)   = c * ( f1 - f2 + f3 - f4)   (rotor drag reaction)
 * </pre>
 * with motors 1=front-right(CCW) 2=rear-right(CW) 3=rear-left(CCW) 4=front-left(CW).
 * {@link FlightController} inverts exactly this model, so commanded torque equals
 * actual torque while the motors are unsaturated. Motors have a first-order spin-up
 * lag. Integration is semi-implicit Euler (rates first, then the quaternion).
 */
public final class QuadDynamics {
    // Physical parameters (small ~0.5 kg quad).
    public final float mass = 0.5f;
    public final float g = 9.81f;
    public final float armLength = 0.15f;            // hub -> motor
    public final float d = armLength / (float) Math.sqrt(2); // axis offset in X layout
    public final float cYaw = 0.02f;                 // yaw torque per N of thrust
    public final float maxThrust = 4.0f;             // per motor (N)
    public final float Ix = 0.0045f, Iy = 0.0045f, Iz = 0.0085f;
    public final float rotDamp = 0.0008f;            // aerodynamic rate damping
    public final float motorTau = 0.02f;             // motor spin-up time constant
    public final float linDampV = 0.10f;             // vertical aerodynamic drag

    private final Quaternion q = new Quaternion();   // body orientation
    private float wx, wy, wz;                          // body rates (rad/s)
    private final float[] motor = new float[4];        // actual thrust per motor (N)
    private float altitude;                            // height above ground (m)
    private float vz;                                  // vertical speed (m/s)

    private final Quaternion wq = new Quaternion();    // scratch
    private final Quaternion qd = new Quaternion();    // scratch

    public float hoverThrust() {
        return mass * g;
    }

    /** Advance one fixed step with commanded per-motor thrusts (N). */
    public void step(float h, float[] thrustCmd) {
        // Motor first-order lag toward command (clamped to physical range).
        float a = Math.min(1f, h / motorTau);
        for (int i = 0; i < 4; i++) {
            float cmd = clamp(thrustCmd[i], 0f, maxThrust);
            motor[i] += (cmd - motor[i]) * a;
        }

        float f1 = motor[0], f2 = motor[1], f3 = motor[2], f4 = motor[3];
        float tx = d * (f1 + f2 - f3 - f4);
        float ty = d * (-f1 + f2 + f3 - f4);
        float tz = cYaw * (f1 - f2 + f3 - f4);

        // Vertical translation: only the body-up (sim +z) component of total thrust
        // lifts; tilting reduces it, so the altitude loop must add throttle to hold.
        float fTotal = f1 + f2 + f3 + f4;
        float upComp = 1f - 2f * (q.x * q.x + q.y * q.y); // body +z vertical component
        float az = fTotal * upComp / mass - g - linDampV * vz / mass;
        vz += az * h;
        altitude += vz * h;
        if (altitude < 0f) { // rest on the ground
            altitude = 0f;
            if (vz < 0f) {
                vz = 0f;
            }
        }

        // Euler's rigid-body equations (diagonal inertia) + linear rate damping.
        float dwx = (tx - (Iz - Iy) * wy * wz - rotDamp * wx) / Ix;
        float dwy = (ty - (Ix - Iz) * wz * wx - rotDamp * wy) / Iy;
        float dwz = (tz - (Iy - Ix) * wx * wy - rotDamp * wz) / Iz;
        wx += dwx * h;
        wy += dwy * h;
        wz += dwz * h;

        // Quaternion kinematics: qdot = 0.5 * q ⊗ (0, w).
        wq.set(0, wx, wy, wz);
        Quaternion.mul(q, wq, qd);
        q.set(q.w + 0.5f * qd.w * h,
              q.x + 0.5f * qd.x * h,
              q.y + 0.5f * qd.y * h,
              q.z + 0.5f * qd.z * h).normalize();
    }

    /** Instantaneous angular-velocity disturbance (rad/s), e.g. a gust kick. */
    public void applyRateImpulse(float dpx, float dpy, float dpz) {
        wx += dpx;
        wy += dpy;
        wz += dpz;
    }

    public void setAttitude(Quaternion attitude) {
        q.set(attitude).normalize();
    }

    public void reset() {
        q.identity();
        wx = wy = wz = 0;
        altitude = 0;
        vz = 0;
        for (int i = 0; i < 4; i++) {
            motor[i] = 0;
        }
    }

    public Quaternion attitude() {
        return q;
    }

    public float rateX() { return wx; }
    public float rateY() { return wy; }
    public float rateZ() { return wz; }
    public float altitude() { return altitude; }
    public float verticalSpeed() { return vz; }

    /** Actual per-motor thrust normalized to 0..1. */
    public float motorFraction(int i) {
        return motor[i] / maxThrust;
    }

    private static float clamp(float v, float lo, float hi) {
        return v < lo ? lo : Math.min(v, hi);
    }
}
