package com.kagenou.Avionics.sim;

/**
 * Online step-response analysis for one channel, the in-sim equivalent of MATLAB's
 * {@code stepinfo}: rise time (10%→90%), percent overshoot, and settling time
 * (last excursion outside a ±5% band around the target). Armed when a step is
 * commanded; fed one sample per sim tick.
 */
public final class StepAnalyzer {
    private static final float SETTLE_BAND = 0.05f; // fraction of step magnitude

    private boolean active;
    private float start, target, span, t0;
    private float maxProg;          // peak progress toward target (1.0 == on target)
    private float riseStartT, riseEndT;
    private boolean got10, got90;
    private float lastOutsideT;     // last time outside the settle band

    public void arm(float startVal, float targetVal, float tNow) {
        active = true;
        start = startVal;
        target = targetVal;
        span = targetVal - startVal;
        t0 = tNow;
        maxProg = 0;
        riseStartT = riseEndT = tNow;
        got10 = got90 = false;
        lastOutsideT = tNow;
    }

    public void sample(float value, float tNow) {
        if (!active) {
            return;
        }
        if (Math.abs(span) < 1e-3f) { // negligible step, nothing meaningful to measure
            active = false;
            return;
        }
        float prog = (value - start) / span; // 1.0 at target
        if (prog > maxProg) {
            maxProg = prog;
        }
        if (!got10 && prog >= 0.1f) {
            riseStartT = tNow;
            got10 = true;
        }
        if (!got90 && prog >= 0.9f) {
            riseEndT = tNow;
            got90 = true;
        }
        if (Math.abs(value - target) > SETTLE_BAND * Math.abs(span)) {
            lastOutsideT = tNow;
        }
    }

    public void clear() {
        active = false;
    }

    public boolean isActive() {
        return active;
    }

    public float riseMs() {
        return got90 ? (riseEndT - riseStartT) * 1000f : Float.NaN;
    }

    public float overshootPct() {
        return Math.max(0f, maxProg - 1f) * 100f;
    }

    public float settleS() {
        return lastOutsideT - t0;
    }
}
