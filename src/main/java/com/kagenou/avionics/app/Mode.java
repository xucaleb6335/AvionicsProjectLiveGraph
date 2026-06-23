package com.kagenou.avionics.app;

import com.kagenou.avionics.math.Quaternion;
import com.kagenou.avionics.sim.SimStatus;

/**
 * A selectable application mode. The render loop drives the active mode and reuses
 * the same scene/HUD to display whatever attitude it produces, so adding a mode is
 * just implementing this interface and registering it.
 */
public interface Mode {
    String name();

    /** Advance mode logic by {@code dt} seconds (skipped while paused). */
    void update(double dt);

    /** Orientation to display this frame. */
    Quaternion attitude();

    /** Mode-specific status line for the banner (without the mode name prefix). */
    String bannerText();

    float[] bannerColor();

    /** Handle a mode-specific key (fired on press and key-repeat). */
    void onKey(int key);

    /** Simulator telemetry for the HUD, or {@code null} if this mode has none. */
    default SimStatus simStatus() {
        return null;
    }

    /** Current attitude setpoint (roll,pitch,yaw deg) for the graph overlay, or {@code null}. */
    default float[] setpointEulerDeg() {
        return null;
    }

    /** World translation (GL units) for the drone + chase camera this frame. */
    default float[] renderOffset() {
        return new float[]{0f, 0f, 0f};
    }

    /** Release any resources (e.g. serial ports) at shutdown. */
    default void dispose() {
    }
}
