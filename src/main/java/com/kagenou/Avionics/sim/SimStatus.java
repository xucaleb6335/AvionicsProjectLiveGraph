package com.kagenou.Avionics.sim;

/**
 * Immutable per-frame telemetry snapshot the HUD renders for the simulator mode:
 * motor outputs, attitude setpoint vs. actual, body rates, the tunable gains (with
 * which one is selected), live step-response metrics, and the sensor-noise flag.
 */
public record SimStatus(
        float[] motors,       // per-motor output 0..1 (length 4)
        float[] setpointDeg,  // roll, pitch, yaw target (length 3)
        float[] attitudeDeg,  // roll, pitch, yaw actual (length 3)
        float[] rateDps,      // body rates deg/s (length 3)
        String[] gainNames,   // tunable gain labels
        float[] gainValues,   // tunable gain values
        int selectedGain,     // index into gainNames/gainValues
        boolean stepActive,   // a step-response measurement is in progress
        float stepRiseMs,     // 10->90% rise time (NaN until reached)
        float stepOvershootPct,
        float stepSettleS,    // time to stay within +/-5% band
        boolean noise,        // sensor-noise injection enabled
        float altitude,       // current altitude (m)
        float altSetpoint) {  // commanded altitude (m)
}
