package com.kagenou.Avionics.app;

import com.kagenou.Avionics.AppConfig;
import com.kagenou.Avionics.io.AttitudeSource;
import com.kagenou.Avionics.io.AttitudeState;
import com.kagenou.Avionics.io.MockSource;
import com.kagenou.Avionics.io.SerialReader;
import com.kagenou.Avionics.math.Quaternion;
import com.kagenou.Avionics.scene.Palette;

import static org.lwjgl.glfw.GLFW.*;

/**
 * The IMU-feed mode: reads the live serial/mock attitude, smooths it, supports
 * recenter (R) and a runtime mock/live swap (M). This is the original visualizer
 * behaviour, now behind the {@link Mode} interface.
 */
public final class LiveMode implements Mode {
    private final AttitudeState state;
    private final AppConfig cfg;
    private AttitudeSource source;
    private boolean useMock;

    private final Quaternion smoothed = new Quaternion();
    private final Quaternion rawTarget = new Quaternion();
    private final Quaternion dispTarget = new Quaternion();
    private final Quaternion offset = new Quaternion(); // recenter: offset ⊗ raw

    public LiveMode(AttitudeState state, AppConfig cfg) {
        this.state = state;
        this.cfg = cfg;
        this.useMock = cfg.demo();
        this.source = makeSource(useMock);
        this.useMock = source instanceof MockSource;
        System.out.println("Java: attitude source = " + source.describe());
        source.start();
    }

    @Override
    public String name() {
        return "LIVE";
    }

    @Override
    public void update(double dt) {
        float[] q = state.snapshot();
        rawTarget.set(q[0], q[1], q[2], q[3]).normalize();
        Quaternion.mul(offset, rawTarget, dispTarget).normalize();
        smoothed.slerpTo(dispTarget, cfg.slerpFactor());
    }

    @Override
    public Quaternion attitude() {
        return smoothed;
    }

    @Override
    public String bannerText() {
        long age = state.ageMillis();
        String label;
        if (age < 0) {
            label = "WAITING - no data yet";
        } else if (age < 500) {
            label = "STREAMING";
        } else if (age < 3000) {
            label = String.format("STALE - no data %.1fs", age / 1000f);
        } else {
            label = String.format("DISCONNECTED - no data %.0fs", age / 1000f);
        }
        return label + "    |    " + source.describe();
    }

    @Override
    public float[] bannerColor() {
        long age = state.ageMillis();
        if (age < 0 || age >= 3000) {
            return age >= 3000 ? Palette.ACCENT_RED : Palette.ACCENT_AMBER;
        }
        return age < 500 ? Palette.ACCENT_GREEN : Palette.ACCENT_AMBER;
    }

    @Override
    public void onKey(int key) {
        switch (key) {
            case GLFW_KEY_R -> { // recenter: zero attitude to the current pose
                float[] q = state.snapshot();
                offset.set(q[0], q[1], q[2], q[3]).normalize().conjugate();
            }
            case GLFW_KEY_M -> swapSource();
            default -> { }
        }
    }

    @Override
    public void dispose() {
        source.stop();
    }

    private void swapSource() {
        source.stop();
        useMock = !useMock;
        source = makeSource(useMock);
        useMock = source instanceof MockSource;
        System.out.println("Java: source -> " + source.describe());
        source.start();
    }

    private AttitudeSource makeSource(boolean mock) {
        if (mock) {
            return new MockSource(state);
        }
        String port = cfg.port() != null ? cfg.port() : SerialReader.autoDetect();
        if (port == null) {
            System.out.println("Java: no serial port found, using mock feed");
            return new MockSource(state);
        }
        return new SerialReader(state, port, cfg.baud());
    }
}
