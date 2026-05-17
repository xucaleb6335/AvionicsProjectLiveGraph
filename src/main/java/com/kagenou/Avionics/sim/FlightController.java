package com.kagenou.Avionics.sim;

import com.kagenou.Avionics.math.Quaternion;

/**
 * Cascaded attitude controller, the structure used by real flight controllers
 * (Betaflight/PX4):
 * <ol>
 *   <li><b>Attitude loop (P):</b> quaternion error between setpoint and measured
 *       attitude → a body-rate setpoint (clamped to a max rate).</li>
 *   <li><b>Rate loop (PID):</b> rate error → body torques.</li>
 *   <li><b>Mixer:</b> desired torque + total thrust → four motor thrusts, the exact
 *       inverse of {@link QuadDynamics}'s forward model.</li>
 * </ol>
 * Roll and pitch share one rate-gain set; yaw has its own (less authority). Gains
 * are public for live tuning.
 */
public final class FlightController {
    // Attitude loop.
    public float attKp = 7.0f;       // (1/s) angle-error -> rate
    public float maxRate = 6.0f;     // rad/s clamp on roll/pitch rate setpoint
    public float maxYawRate = 3.0f;  // rad/s clamp on yaw rate setpoint

    // Rate loop gains (roll/pitch shared).
    public float rateKp = 0.11f;
    public float rateKi = 0.04f;
    public float rateKd = 0.0028f;
    // Yaw rate gains.
    public float yawKp = 0.16f;
    public float yawKi = 0.03f;
    public float yawKd = 0.0f;

    private final Pid rateX = new Pid(0, 0, 0, 0.8f, 0.35f, 0.008f);
    private final Pid rateY = new Pid(0, 0, 0, 0.8f, 0.35f, 0.008f);
    private final Pid rateZ = new Pid(0, 0, 0, 0.15f, 0.06f, 0.008f);

    private final Quaternion conj = new Quaternion();
    private final Quaternion qErr = new Quaternion();
    private final float[] motorOut = new float[4];

    /**
     * @param qMeas   measured attitude
     * @param wx,wy,wz measured body rates (rad/s)
     * @param qSp     attitude setpoint
     * @param thrust  desired total thrust (N), e.g. hover
     * @return four commanded motor thrusts (N), clamped to the motor range
     */
    public float[] update(QuadDynamics quad, Quaternion qMeas, float wx, float wy, float wz,
                          Quaternion qSp, float thrust, float h) {
        pushGains();

        // Attitude error as the rotation from measured to setpoint, in body axes.
        conj.set(qMeas).conjugate();
        Quaternion.mul(conj, qSp, qErr).normalize();
        float sign = qErr.w < 0 ? -1f : 1f;          // take the short way around
        float exRot = 2f * sign * qErr.x;
        float eyRot = 2f * sign * qErr.y;
        float ezRot = 2f * sign * qErr.z;

        float wxSp = clamp(attKp * exRot, -maxRate, maxRate);
        float wySp = clamp(attKp * eyRot, -maxRate, maxRate);
        float wzSp = clamp(attKp * ezRot, -maxYawRate, maxYawRate);

        // Rate loop -> torques.
        float tx = rateX.update(wxSp, wx, h);
        float ty = rateY.update(wySp, wy, h);
        float tz = rateZ.update(wzSp, wz, h);

        // Mixer: inverse of the dynamics' torque model (rows are orthogonal).
        float d = quad.d, c = quad.cYaw;
        float base = thrust / 4f;
        float rx = tx / (4f * d), ry = ty / (4f * d), rz = tz / (4f * c);
        motorOut[0] = base + rx - ry + rz;
        motorOut[1] = base + rx + ry - rz;
        motorOut[2] = base - rx + ry + rz;
        motorOut[3] = base - rx - ry - rz;
        for (int i = 0; i < 4; i++) {
            motorOut[i] = clamp(motorOut[i], 0f, quad.maxThrust);
        }
        return motorOut;
    }

    public void reset() {
        rateX.reset();
        rateY.reset();
        rateZ.reset();
    }

    private void pushGains() {
        rateX.kp = rateKp; rateX.ki = rateKi; rateX.kd = rateKd;
        rateY.kp = rateKp; rateY.ki = rateKi; rateY.kd = rateKd;
        rateZ.kp = yawKp;  rateZ.ki = yawKi;  rateZ.kd = yawKd;
    }

    private static float clamp(float v, float lo, float hi) {
        return v < lo ? lo : Math.min(v, hi);
    }
}
