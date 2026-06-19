package com.kagenou.Avionics;

import com.kagenou.Avionics.app.LiveMode;
import com.kagenou.Avionics.app.Mode;
import com.kagenou.Avionics.app.SimMode;
import com.kagenou.Avionics.hud.Hud;
import com.kagenou.Avionics.io.AttitudeState;
import com.kagenou.Avionics.math.Quaternion;
import com.kagenou.Avionics.scene.Background;
import com.kagenou.Avionics.scene.Drone;
import com.kagenou.Avionics.scene.Ground;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;

public class Main {
    public static void main(String[] args) {
        AppConfig cfg = AppConfig.fromArgs(args);

        AttitudeState state = new AttitudeState();
        Mode[] modes = {new LiveMode(state, cfg), new SimMode()};
        int[] mode = {0}; // index of active mode (held in array for the key callback)

        Engine engine = new Engine(cfg);
        Background background = new Background();
        Ground ground = new Ground();
        Drone drone = new Drone();
        Cube cube = new Cube();
        Hud hud = new Hud(cfg.historySeconds());
        Controls controls = new Controls();

        installKeybinds(engine.window(), controls, modes, mode);
        printHelp();

        float[] matrix = new float[16];
        long startNanos = System.nanoTime();
        long lastFrameNanos = startNanos;
        long fpsNanos = startNanos;
        int frames = 0;
        double fps = 0;

        while (!glfwWindowShouldClose(engine.window())) {
            long nowNanos = System.nanoTime();
            double frameDt = (nowNanos - lastFrameNanos) / 1e9;
            lastFrameNanos = nowNanos;

            Mode active = modes[mode[0]];
            if (!controls.paused) {
                active.update(frameDt);
            }
            Quaternion displayed = active.attitude();
            displayed.toMatrixColumnMajor(matrix);
            hud.record(displayed, active.setpointEulerDeg()); // feed the time-series history every frame

            float[] off = active.renderOffset();
            engine.setCameraTarget(off[0], off[1], off[2]); // chase the drone

            engine.beginFrame();
            background.draw();
            if (controls.showGround) {
                ground.draw();
            }

            glPushMatrix();
            glTranslatef(off[0], off[1], off[2]); // world position
            glMultMatrixf(matrix); // apply attitude
            if (controls.showDrone) {
                glEnable(GL_LIGHTING);
                drone.draw((nowNanos - startNanos) / 1e9);
                glDisable(GL_LIGHTING);
            } else {
                cube.draw();
            }
            glPopMatrix();

            frames++;
            double fdt = (System.nanoTime() - fpsNanos) / 1e9;
            if (fdt >= 0.5) {
                fps = frames / fdt;
                frames = 0;
                fpsNanos = System.nanoTime();
            }

            if (controls.showHud) {
                hud.draw(engine.fbWidth(), engine.fbHeight(), state, displayed, fps,
                        active.name(), active.bannerText(), active.bannerColor(), active.simStatus(), controls);
            }

            engine.endFrame();
        }

        for (Mode m : modes) {
            m.dispose();
        }
        engine.terminate();
    }

    private static void installKeybinds(long window, Controls c, Mode[] modes, int[] mode) {
        glfwSetKeyCallback(window, (win, key, scancode, action, mods) -> {
            if (action == GLFW_RELEASE) {
                return;
            }
            // Global keys act on initial press only; if not consumed (or on repeat),
            // forward to the active mode (so held arrows/+- ramp smoothly).
            if (action == GLFW_PRESS && handleGlobal(win, key, c, modes, mode)) {
                return;
            }
            modes[mode[0]].onKey(key);
        });
    }

    private static boolean handleGlobal(long win, int key, Controls c, Mode[] modes, int[] mode) {
        switch (key) {
            case GLFW_KEY_ESCAPE -> glfwSetWindowShouldClose(win, true);
            case GLFW_KEY_TAB -> mode[0] = (mode[0] + 1) % modes.length;
            case GLFW_KEY_H -> c.showHud = !c.showHud;
            case GLFW_KEY_1 -> c.showAttitude = !c.showAttitude;
            case GLFW_KEY_2 -> c.showDiagnostics = !c.showDiagnostics;
            case GLFW_KEY_3 -> c.showGraph = !c.showGraph;
            case GLFW_KEY_4 -> c.showHints = !c.showHints;
            case GLFW_KEY_C -> c.showDrone = !c.showDrone;
            case GLFW_KEY_G -> c.showGround = !c.showGround;
            case GLFW_KEY_SPACE -> c.paused = !c.paused;
            default -> {
                return false;
            }
        }
        return true;
    }

    private static void printHelp() {
        System.out.println("Java: Tab switches mode (LIVE / SIM).");
        System.out.println("Java: global - H hud | 1 attitude | 2 panel | 3 graph | 4 hints | C drone/cube | G grid | Space pause | Esc quit");
        System.out.println("Java: LIVE   - R recenter | M mock/live");
        System.out.println("Java: SIM    - arrows tilt | Q/E yaw | T roll-step | Z gust | 0 level | R reset | [ ] select gain | - = adjust");
    }
}
