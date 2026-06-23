package com.kagenou.avionics.sim;

/**
 * One-axis PID controller sized for a flight inner loop:
 * <ul>
 *   <li>integral clamped so the I-term cannot exceed a bound (anti-windup),</li>
 *   <li>derivative taken on the <em>measurement</em> (not the error) and low-pass
 *       filtered, so a setpoint step doesn't produce a derivative kick,</li>
 *   <li>output saturation.</li>
 * </ul>
 * Gains are public so a tuning UI can change them live.
 */
public final class Pid {
    public float kp, ki, kd;

    private final float outMax;
    private final float iTermMax;   // bound on |ki * integral|
    private final float derivTau;   // derivative low-pass time constant (s)

    private float integral;
    private float prevMeas;
    private boolean hasPrev;
    private float dState;            // filtered derivative of measurement

    public Pid(float kp, float ki, float kd, float outMax, float iTermMax, float derivTau) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.outMax = outMax;
        this.iTermMax = iTermMax;
        this.derivTau = derivTau;
    }

    public float update(float setpoint, float meas, float h) {
        float err = setpoint - meas;

        integral += err * h;
        if (ki > 1e-9f) {
            float iLim = iTermMax / ki;
            integral = clamp(integral, -iLim, iLim);
        }

        float rawDeriv = hasPrev ? -(meas - prevMeas) / h : 0f; // d(error)/dt with constant setpoint
        prevMeas = meas;
        hasPrev = true;
        float alpha = h / (derivTau + h);
        dState += (rawDeriv - dState) * alpha;

        float out = kp * err + ki * integral + kd * dState;
        return clamp(out, -outMax, outMax);
    }

    public void reset() {
        integral = 0;
        dState = 0;
        hasPrev = false;
    }

    private static float clamp(float v, float lo, float hi) {
        return v < lo ? lo : Math.min(v, hi);
    }
}
