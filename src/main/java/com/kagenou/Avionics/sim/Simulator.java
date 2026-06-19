package com.kagenou.avionics.sim;

import com.kagenou.avionics.math.Quaternion;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Random;

/**
 * Drives {@link FlightController} + {@link QuadDynamics} at a fixed internal rate,
 * decoupled from frame rate via an accumulator. Holds the attitude setpoint and
 * exposes telemetry plus a small list of tunable gains for the HUD.
 *
 * <p>Only rotational dynamics are modeled (the quad holds a hover thrust and spins
 * in place), which is exactly what's needed to evaluate the attitude/rate PID and
 * the motor mixer.
 */
public final class Simulator {
    private static final float STEP = 0.002f;          // 500 Hz inner loop
    private static final float MAX_FRAME = 0.1f;       // clamp to avoid spiral-of-death
    private static final float MAX_TILT = 60f;         // setpoint clamp (deg)

    private static final float RATE_NOISE = 0.06f;   // rad/s std-dev of injected gyro noise
    private static final float[] RATE_BIAS = {0.02f, -0.015f, 0.01f};

    private final QuadDynamics quad = new QuadDynamics();
    private final FlightController fc = new FlightController();
    private final StepAnalyzer analyzer = new StepAnalyzer();
    private final Random rng = new Random();
    // Altitude-hold loop: thrust = hover feed-forward + PD(altitude) (D damps via -Kd*vz).
    private final Pid altPid = new Pid(5f, 0.5f, 3f, 11f, 4f, 0.05f);
    private float altSetpoint = 1.5f;

    private final Quaternion qSp = new Quaternion();
    private float spRollDeg, spPitchDeg, spYawDeg;
    private float acc;
    private float simTime;
    private int stepChannel;
    private boolean noise;
    private int currentPreset = 4; // defaults equal the "Tuned PID" preset

    private static final float D2R = (float) (Math.PI / 180.0);
    private static final float R2D = (float) (180.0 / Math.PI);

    public static final String[] GAIN_NAMES = {"att Kp", "rate Kp", "rate Ki", "rate Kd", "yaw Kp", "alt Kp", "alt Kd"};
    private static final float[] GAIN_STEP = {0.5f, 0.01f, 0.005f, 0.0005f, 0.01f, 0.5f, 0.5f};
    private static final float[] GAIN_MIN = {0f, 0f, 0f, 0f, 0f, 0f, 0f};
    private static final float[] GAIN_MAX = {20f, 0.4f, 0.4f, 0.02f, 0.4f, 15f, 10f};

    public static final String[] PRESET_NAMES = {
            "Open Loop", "P Only", "PI", "PD", "Tuned PID", "Oscillating", "I Windup", "Noisy D"};
    // Rows: {att Kp, rate Kp, rate Ki, rate Kd, yaw Kp, alt Kp, alt Kd}. Altitude is kept
    // tuned in every preset except Open Loop so the demo isolates the attitude/rate loop.
    private static final float[][] PRESET_GAINS = {
            {0f,  0f,    0f,    0f,      0f,    0f, 0f},   // Open Loop  - no feedback
            {6f,  0.10f, 0f,    0f,      0.12f, 5f, 3f},   // P Only     - steady-state error
            {6f,  0.10f, 0.10f, 0f,      0.14f, 5f, 3f},   // PI         - no error, oscillatory
            {7f,  0.11f, 0f,    0.0040f, 0.15f, 5f, 3f},   // PD         - damped, slight offset
            {7f,  0.11f, 0.04f, 0.0028f, 0.16f, 5f, 3f},   // Tuned PID  - the good values
            {20f, 0.12f, 0.03f, 0f,      0.20f, 5f, 3f},   // Oscillating- fast outer, slow inner, no D
            {5f,  0.04f, 0.35f, 0f,      0.10f, 5f, 3f},   // I Windup   - dominant integral
            {7f,  0.11f, 0.04f, 0.0150f, 0.16f, 5f, 3f},   // Noisy D    - high D + sensor noise
    };
    private static final boolean[] PRESET_NOISE = {false, false, false, false, false, false, false, true};

    public Simulator() {
        updateSetpoint();
    }

    public void step(double dt) {
        acc += (float) Math.min(dt, MAX_FRAME);
        while (acc >= STEP) {
            float wx = quad.rateX(), wy = quad.rateY(), wz = quad.rateZ();
            if (noise) { // gyro noise + bias on the rates fed to the controller
                wx += (float) rng.nextGaussian() * RATE_NOISE + RATE_BIAS[0];
                wy += (float) rng.nextGaussian() * RATE_NOISE + RATE_BIAS[1];
                wz += (float) rng.nextGaussian() * RATE_NOISE + RATE_BIAS[2];
            }
            float thrust = quad.hoverThrust() + altPid.update(altSetpoint, quad.altitude(), STEP);
            thrust = clamp(thrust, 0f, 4f * quad.maxThrust);
            float[] cmd = fc.update(quad, quad.attitude(), wx, wy, wz, qSp, thrust, STEP);
            quad.step(STEP, cmd);
            acc -= STEP;
            simTime += STEP;
        }
        if (analyzer.isActive()) {
            analyzer.sample(quad.attitude().toEulerDeg()[stepChannel], simTime);
        }
    }

    public Quaternion attitude() {
        return quad.attitude();
    }

    // --- setpoint ---

    public void setSetpoint(float rollDeg, float pitchDeg, float yawDeg) {
        spRollDeg = clamp(rollDeg, -MAX_TILT, MAX_TILT);
        spPitchDeg = clamp(pitchDeg, -MAX_TILT, MAX_TILT);
        spYawDeg = wrap180(yawDeg);
        updateSetpoint();
    }

    public void nudgeSetpoint(float dRoll, float dPitch, float dYaw) {
        setSetpoint(spRollDeg + dRoll, spPitchDeg + dPitch, spYawDeg + dYaw);
    }

    public void levelSetpoint() {
        setSetpoint(0, 0, spYawDeg);
    }

    public void nudgeAltitude(float d) {
        altSetpoint = clamp(altSetpoint + d, 0.2f, 6f);
    }

    public float altitude() {
        return quad.altitude();
    }

    public float altSetpoint() {
        return altSetpoint;
    }

    /** Forces the simulated attitude (e.g. to seed a disturbance-recovery test). */
    public void setAttitudeEuler(float rollDeg, float pitchDeg, float yawDeg) {
        quad.setAttitude(Quaternion.fromEuler(rollDeg * D2R, pitchDeg * D2R, yawDeg * D2R));
    }

    private void updateSetpoint() {
        qSp.set(Quaternion.fromEuler(spRollDeg * D2R, spPitchDeg * D2R, spYawDeg * D2R));
    }

    // --- disturbances / reset ---

    public void disturb() {
        quad.applyRateImpulse(3.5f, -2.5f, 1.5f);
    }

    /** Commands a roll step to {@code targetDeg} and arms the step-response analyzer. */
    public void commandStep(float targetDeg) {
        float[] e = quad.attitude().toEulerDeg();
        stepChannel = 0; // roll
        spRollDeg = clamp(targetDeg, -MAX_TILT, MAX_TILT);
        updateSetpoint();
        analyzer.arm(e[0], spRollDeg, simTime);
    }

    public void toggleNoise() {
        noise = !noise;
    }

    public boolean isNoise() {
        return noise;
    }

    public float[] setpointEulerDeg() {
        return new float[]{spRollDeg, spPitchDeg, spYawDeg};
    }

    /** Writes the current gains as C {@code #define}s (for firmware) and returns the text. */
    public String exportGains() {
        String text = """
                // Tuned PID gains exported from the simulator
                #define ATT_KP   %.4ff
                #define RATE_KP  %.4ff
                #define RATE_KI  %.4ff
                #define RATE_KD  %.4ff
                #define YAW_KP   %.4ff
                """.formatted(gainValue(0), gainValue(1), gainValue(2), gainValue(3), gainValue(4));
        try {
            Files.writeString(Path.of("tuned-gains.h"), text);
            System.out.println("Java: wrote tuned-gains.h");
        } catch (IOException e) {
            System.out.println("Java: could not write tuned-gains.h: " + e.getMessage());
        }
        System.out.println(text);
        return text;
    }

    public void reset() {
        quad.reset();
        fc.reset();
        altPid.reset();
        analyzer.clear();
        acc = 0;
    }

    // --- tunable gains ---

    public int gainCount() {
        return GAIN_NAMES.length;
    }

    public float gainValue(int i) {
        return switch (i) {
            case 0 -> fc.attKp;
            case 1 -> fc.rateKp;
            case 2 -> fc.rateKi;
            case 3 -> fc.rateKd;
            case 4 -> fc.yawKp;
            case 5 -> altPid.kp;
            case 6 -> altPid.kd;
            default -> 0f;
        };
    }

    public void setGain(int i, float v) {
        v = clamp(v, GAIN_MIN[i], GAIN_MAX[i]);
        switch (i) {
            case 0 -> fc.attKp = v;
            case 1 -> fc.rateKp = v;
            case 2 -> fc.rateKi = v;
            case 3 -> fc.rateKd = v;
            case 4 -> fc.yawKp = v;
            case 5 -> altPid.kp = v;
            case 6 -> altPid.kd = v;
            default -> { }
        }
        currentPreset = -1; // manual change -> custom
    }

    public void adjustGain(int i, int dir) {
        setGain(i, gainValue(i) + dir * GAIN_STEP[i]);
    }

    public float gainMin(int i) {
        return GAIN_MIN[i];
    }

    public float gainMax(int i) {
        return GAIN_MAX[i];
    }

    // --- presets ---

    public String[] presetNames() {
        return PRESET_NAMES;
    }

    public int currentPreset() {
        return currentPreset;
    }

    /** Applies a named preset: sets all gains + noise and resets the integrators. */
    public void applyPreset(int idx) {
        if (idx < 0 || idx >= PRESET_GAINS.length) {
            return;
        }
        float[] g = PRESET_GAINS[idx];
        fc.attKp = g[0];
        fc.rateKp = g[1];
        fc.rateKi = g[2];
        fc.rateKd = g[3];
        fc.yawKp = g[4];
        altPid.kp = g[5];
        altPid.kd = g[6];
        noise = PRESET_NOISE[idx];
        fc.reset();
        altPid.reset();
        analyzer.clear();
        currentPreset = idx;
    }

    public SimStatus status(int selectedGain) {
        float[] motors = {quad.motorFraction(0), quad.motorFraction(1), quad.motorFraction(2), quad.motorFraction(3)};
        float[] e = quad.attitude().toEulerDeg();
        float[] rates = {quad.rateX() * R2D, quad.rateY() * R2D, quad.rateZ() * R2D};
        float[] gains = new float[GAIN_NAMES.length];
        for (int i = 0; i < gains.length; i++) {
            gains[i] = gainValue(i);
        }
        return new SimStatus(motors,
                new float[]{spRollDeg, spPitchDeg, spYawDeg},
                e, rates, GAIN_NAMES, gains, selectedGain,
                analyzer.isActive(), analyzer.riseMs(), analyzer.overshootPct(), analyzer.settleS(),
                noise, quad.altitude(), altSetpoint,
                GAIN_MIN, GAIN_MAX, PRESET_NAMES, currentPreset);
    }

    private static float clamp(float v, float lo, float hi) {
        return v < lo ? lo : Math.min(v, hi);
    }

    private static float wrap180(float deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
