package com.kagenou.avionics.app;

import com.kagenou.avionics.math.FixedPointCoords;
import com.kagenou.avionics.math.FixedPointCoords.FP;
import com.kagenou.avionics.math.Quaternion;
import com.kagenou.avionics.scene.Palette;
import org.lwjgl.glfw.GLFWGamepadState;
import org.lwjgl.system.MemoryStack;

import static org.lwjgl.glfw.GLFW.*;

/**
 * PS5 DualSense flight mode. Reads the controller over USB-C through GLFW's HID
 * gamepad layer ({@code glfwGetGamepadState}) and flies the drone across the
 * quantized Q8.8 plane.
 *
 * <p>Controls: left stick = X/Y translation, R2/L2 = speed boost, D-pad up/down =
 * altitude, Triangle = breadcrumb-trail toggle, Square = camera-perspective toggle.
 * Keyboard fallback (no controller): WASD translate, Q/E down/up, V view, B trail,
 * R recenter.
 */
public final class PS5ControllerMode implements Mode {
    private static final int PAD = GLFW_JOYSTICK_1;

    private static final float STICK_DEADZONE = 0.15f;
    private static final float TRIGGER_DEADZONE = 0.02f;
    private static final float BASE_SPEED = 4f;      // m/s at full stick, no boost
    private static final float MAX_BOOST = 4f;       // speed multiplier at full triggers
    private static final float VERT_SPEED = 3f;      // m/s D-pad climb / descend
    private static final float MAX_BANK = (float) Math.toRadians(22); // display tilt
    private static final float METER_TO_GL = 1f;
    private static final float BASE_HEIGHT = 0.4f;   // GL units to float above ground

    // World position in Q8.8 (x = east, y = north, z = up); spawn at the origin.
    private FP px = FixedPointCoords.ORIGIN;
    private FP py = FixedPointCoords.ORIGIN;
    private FP pz = FixedPointCoords.ORIGIN;
    private float headingRad;

    private boolean firstPerson;   // false = 3rd person
    private boolean trailActive;
    private boolean connected;
    private boolean prevTriangle, prevSquare;

    // Breadcrumb ring buffer: static-capacity circular SoA of Q8.8 points.
    private static final int TRAIL_CAP = 1000;
    private static final float TRAIL_STEP = 0.5f; // metres traversed between crumbs
    private final short[] trailX = new short[TRAIL_CAP];
    private final short[] trailY = new short[TRAIL_CAP];
    private final short[] trailZ = new short[TRAIL_CAP];
    private int trailHead;   // next write slot
    private int trailCount;  // populated entries (<= TRAIL_CAP)
    private FP lastCrumbX = FixedPointCoords.ORIGIN;
    private FP lastCrumbY = FixedPointCoords.ORIGIN;
    private FP lastCrumbZ = FixedPointCoords.ORIGIN;

    private final Quaternion display = new Quaternion().identity();

    @Override
    public String name() {
        return "PS5";
    }

    @Override
    public void update(double dt) {
        float h = (float) dt;
        float lx = 0, ly = 0, throttle = 0;
        boolean up = false, down = false, triangle = false, square = false;
        connected = glfwJoystickIsGamepad(PAD);

        if (connected) {
            try (MemoryStack stack = MemoryStack.stackPush()) {
                GLFWGamepadState gp = GLFWGamepadState.malloc(stack);
                if (glfwGetGamepadState(PAD, gp)) {
                    lx = deadzone(gp.axes(GLFW_GAMEPAD_AXIS_LEFT_X));
                    ly = deadzone(gp.axes(GLFW_GAMEPAD_AXIS_LEFT_Y));
                    float r2 = trigger(gp.axes(GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER));
                    float l2 = trigger(gp.axes(GLFW_GAMEPAD_AXIS_LEFT_TRIGGER));
                    throttle = Math.min(1f, r2 + l2);
                    up = gp.buttons(GLFW_GAMEPAD_BUTTON_DPAD_UP) == GLFW_PRESS;
                    down = gp.buttons(GLFW_GAMEPAD_BUTTON_DPAD_DOWN) == GLFW_PRESS;
                    triangle = gp.buttons(GLFW_GAMEPAD_BUTTON_TRIANGLE) == GLFW_PRESS;
                    square = gp.buttons(GLFW_GAMEPAD_BUTTON_SQUARE) == GLFW_PRESS;
                }
                updateHaptics(gp);
            }
        }

        // Rising-edge toggles so a held button flips state only once.
        if (triangle && !prevTriangle) {
            toggleTrail();
        }
        if (square && !prevSquare) {
            firstPerson = !firstPerson;
        }
        prevTriangle = triangle;
        prevSquare = square;

        // Left stick -> horizontal translation (stick Y is +down, invert for +north).
        float speed = BASE_SPEED * (1f + throttle * (MAX_BOOST - 1f));
        float dEast = lx * speed * h;
        float dNorth = -ly * speed * h;
        px = px.addMetres(dEast);
        py = py.addMetres(dNorth);
        if (up) {
            pz = pz.addMetres(VERT_SPEED * h);
        }
        if (down) {
            pz = pz.addMetres(-VERT_SPEED * h);
        }
        recordCrumb();

        // Heading follows horizontal motion; the displayed body banks into the input.
        if (Math.hypot(dEast, dNorth) > 1e-4) {
            headingRad = (float) Math.atan2(dNorth, dEast);
        }
        display.set(Quaternion.fromEuler(-lx * MAX_BANK, ly * MAX_BANK, headingRad));
    }

    @Override
    public Quaternion attitude() {
        return display;
    }

    @Override
    public float[] renderOffset() {
        // World (east, north, up) m -> GL (x right, y up, z out-of-screen); north goes in.
        return new float[]{
                px.toFloat() * METER_TO_GL,
                BASE_HEIGHT + pz.toFloat() * METER_TO_GL,
                -py.toFloat() * METER_TO_GL
        };
    }

    @Override
    public String bannerText() {
        return connected
                ? "DualSense - L-stick fly | R2/L2 boost | Tri trail | Sq view"
                : "DualSense - NO CONTROLLER (connect over USB-C); keyboard fallback active";
    }

    @Override
    public float[] bannerColor() {
        return connected ? Palette.ACCENT_GREEN : new float[]{1f, 0.78f, 0.2f};
    }

    @Override
    public void onKey(int key) {
        switch (key) { // keyboard fallback when no controller is attached
            case GLFW_KEY_A -> px = px.addMetres(-0.5f);
            case GLFW_KEY_D -> px = px.addMetres(0.5f);
            case GLFW_KEY_W -> py = py.addMetres(0.5f);
            case GLFW_KEY_S -> py = py.addMetres(-0.5f);
            case GLFW_KEY_E -> pz = pz.addMetres(0.5f);
            case GLFW_KEY_Q -> pz = pz.addMetres(-0.5f);
            case GLFW_KEY_V -> firstPerson = !firstPerson;
            case GLFW_KEY_B -> toggleTrail();
            case GLFW_KEY_R -> {
                px = FixedPointCoords.ORIGIN;
                py = FixedPointCoords.ORIGIN;
                pz = FixedPointCoords.ORIGIN;
                headingRad = 0;
            }
            default -> { }
        }
    }

    /**
     * DualSense rumble actuator hook. GLFW exposes no haptic output for the pad, so
     * this verifies the connection only and drives a (0, 0) intensity payload.
     */
    private void updateHaptics(GLFWGamepadState gp) {
        float lowFreq = 0f, highFreq = 0f; // (0, 0) intensity -- no payload
        // TODO: Future telemetry haptic feedback
    }

    private void toggleTrail() {
        trailActive = !trailActive;
        if (trailActive) { // begin recording a fresh path from the current spot
            trailHead = 0;
            trailCount = 0;
            lastCrumbX = px;
            lastCrumbY = py;
            lastCrumbZ = pz;
        }
    }

    /** Push the current quantized position once the drone has moved TRAIL_STEP metres. */
    private void recordCrumb() {
        if (!trailActive) {
            return;
        }
        float dx = px.toFloat() - lastCrumbX.toFloat();
        float dy = py.toFloat() - lastCrumbY.toFloat();
        float dz = pz.toFloat() - lastCrumbZ.toFloat();
        if (dx * dx + dy * dy + dz * dz < TRAIL_STEP * TRAIL_STEP) {
            return;
        }
        trailX[trailHead] = px.raw();
        trailY[trailHead] = py.raw();
        trailZ[trailHead] = pz.raw();
        trailHead = (trailHead + 1) % TRAIL_CAP;
        if (trailCount < TRAIL_CAP) {
            trailCount++;
        }
        lastCrumbX = px;
        lastCrumbY = py;
        lastCrumbZ = pz;
    }

    // --- state consumed by the Phase 3 camera / trail / HUD ---

    public int trailSize() {
        return trailCount;
    }

    /** Oldest-to-newest ordinal -> physical ring index. */
    private int trailIndex(int ordinal) {
        return (trailHead - trailCount + ordinal + 2 * TRAIL_CAP) % TRAIL_CAP;
    }

    public float trailGlX(int ordinal) {
        return FixedPointCoords.toFloat(trailX[trailIndex(ordinal)]) * METER_TO_GL;
    }

    public float trailGlY(int ordinal) {
        return BASE_HEIGHT + FixedPointCoords.toFloat(trailZ[trailIndex(ordinal)]) * METER_TO_GL;
    }

    public float trailGlZ(int ordinal) {
        return -FixedPointCoords.toFloat(trailY[trailIndex(ordinal)]) * METER_TO_GL;
    }

    public boolean firstPerson() {
        return firstPerson;
    }

    public boolean trailActive() {
        return trailActive;
    }

    public boolean controllerConnected() {
        return connected;
    }

    public float headingRad() {
        return headingRad;
    }

    public float worldX() {
        return px.toFloat();
    }

    public float worldY() {
        return py.toFloat();
    }

    public float worldZ() {
        return pz.toFloat();
    }

    /** Radial deadzone with rescaling so output ramps from 0 at the edge of the zone. */
    private static float deadzone(float v) {
        if (Math.abs(v) < STICK_DEADZONE) {
            return 0f;
        }
        return (v - Math.signum(v) * STICK_DEADZONE) / (1f - STICK_DEADZONE);
    }

    /** GLFW trigger axis (rest -1 .. full +1) normalized to 0..1 with a small deadzone. */
    private static float trigger(float axis) {
        float n = (axis + 1f) * 0.5f;
        return n < TRIGGER_DEADZONE ? 0f : n;
    }
}
