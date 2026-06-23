package com.kagenou.avionics.app;

import com.kagenou.avionics.math.Quaternion;
import com.kagenou.avionics.scene.Palette;
import com.kagenou.avionics.sim.SimStatus;
import com.kagenou.avionics.sim.Simulator;

import static org.lwjgl.glfw.GLFW.*;

/**
 * The flight-controller simulator mode: a physics quad stabilized by a cascaded
 * PID. You drive the attitude setpoint, inject disturbances, and tune the PID gains
 * live to evaluate the motor-control response (shown on the drone, the sim panel,
 * and the time-series graph).
 *
 * <p>Keys: arrows tilt (roll/pitch), W/S altitude, Q/E yaw, T 20-deg roll step
 * (with response analysis), Z gust, N sensor-noise toggle, P export gains, 0 level,
 * R/Backspace reset, [ ] select gain, - = adjust selected gain.
 */
public final class SimMode implements Mode {
    private static final float GROUND_Y = -0.6f;   // matches the scene Ground plane
    private static final float ALT_SCALE = 0.45f;  // metres -> scene units for rendering

    private final Simulator sim = new Simulator();
    private int selectedGain = 1; // start on rate Kp

    @Override
    public String name() {
        return "SIM";
    }

    @Override
    public void update(double dt) {
        sim.step(dt);
    }

    @Override
    public Quaternion attitude() {
        return sim.attitude();
    }

    @Override
    public SimStatus simStatus() {
        return sim.status(selectedGain);
    }

    /** Exposes the simulator so the HUD can drive sliders and presets directly. */
    public Simulator simulator() {
        return sim;
    }

    @Override
    public String bannerText() {
        return sim.isNoise() ? "cascaded PID - SENSOR NOISE ON" : "cascaded PID - attitude + rate loops";
    }

    @Override
    public float[] setpointEulerDeg() {
        return sim.setpointEulerDeg();
    }

    @Override
    public float[] renderOffset() {
        // Lift the drone (and the chase camera) by its altitude; horizontal stays at origin.
        return new float[]{0f, GROUND_Y + 0.15f + sim.altitude() * ALT_SCALE, 0f};
    }

    @Override
    public float[] bannerColor() {
        return Palette.ACCENT_GREEN;
    }

    @Override
    public void onKey(int key) {
        switch (key) {
            case GLFW_KEY_LEFT -> sim.nudgeSetpoint(-5, 0, 0);
            case GLFW_KEY_RIGHT -> sim.nudgeSetpoint(5, 0, 0);
            case GLFW_KEY_UP -> sim.nudgeSetpoint(0, 5, 0);
            case GLFW_KEY_DOWN -> sim.nudgeSetpoint(0, -5, 0);
            case GLFW_KEY_Q -> sim.nudgeSetpoint(0, 0, -10);
            case GLFW_KEY_E -> sim.nudgeSetpoint(0, 0, 10);
            case GLFW_KEY_W -> sim.nudgeAltitude(0.3f);
            case GLFW_KEY_S -> sim.nudgeAltitude(-0.3f);
            case GLFW_KEY_T -> sim.commandStep(20); // 20-deg roll step + analyze
            case GLFW_KEY_0 -> sim.levelSetpoint();
            case GLFW_KEY_Z -> sim.disturb();
            case GLFW_KEY_N -> sim.toggleNoise();
            case GLFW_KEY_P -> sim.exportGains();
            case GLFW_KEY_BACKSPACE, GLFW_KEY_R -> sim.reset();
            case GLFW_KEY_LEFT_BRACKET -> selectedGain = (selectedGain - 1 + sim.gainCount()) % sim.gainCount();
            case GLFW_KEY_RIGHT_BRACKET -> selectedGain = (selectedGain + 1) % sim.gainCount();
            case GLFW_KEY_MINUS -> sim.adjustGain(selectedGain, -1);
            case GLFW_KEY_EQUAL -> sim.adjustGain(selectedGain, +1);
            default -> { }
        }
    }
}
