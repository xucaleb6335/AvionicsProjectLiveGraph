package com.kagenou.avionics;

import com.kagenou.avionics.app.LiveMode;
import com.kagenou.avionics.app.Mode;
import com.kagenou.avionics.app.PS5ControllerMode;
import com.kagenou.avionics.app.SimMode;
import com.kagenou.avionics.core.AppConfig;
import com.kagenou.avionics.core.Controls;
import com.kagenou.avionics.core.Engine;
import com.kagenou.avionics.hud.Hud;
import com.kagenou.avionics.io.AttitudeState;
import com.kagenou.avionics.math.Quaternion;
import com.kagenou.avionics.scene.Background;
import com.kagenou.avionics.scene.Cube;
import com.kagenou.avionics.scene.Drone;
import com.kagenou.avionics.scene.Ground;
import com.kagenou.avionics.sim.Simulator;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;

public class Main {
    public static void main(String[] args) {
        AppConfig cfg = AppConfig.fromArgs(args);

        AttitudeState state = new AttitudeState();
        Mode[] modes = {new LiveMode(state, cfg), new SimMode(), new PS5ControllerMode()};
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

            // Poll the mouse, scaled into HUD/framebuffer coords, for sliders + presets.
            double[] cx = {0}, cy = {0};
            glfwGetCursorPos(engine.window(), cx, cy);
            int[] ww = {0}, wh = {0};
            glfwGetWindowSize(engine.window(), ww, wh);
            controls.mouseX = (float) (cx[0] * (ww[0] > 0 ? (double) engine.fbWidth() / ww[0] : 1));
            controls.mouseY = (float) (cy[0] * (wh[0] > 0 ? (double) engine.fbHeight() / wh[0] : 1));
            boolean mdown = glfwGetMouseButton(engine.window(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            controls.mousePressedEdge = mdown && !controls.mouseDown;
            controls.mouseDown = mdown;

            Mode active = modes[mode[0]];
            Simulator simCtl = active instanceof SimMode sm ? sm.simulator() : null;
            PS5ControllerMode ps = active instanceof PS5ControllerMode m ? m : null;
            if (!controls.paused) {
                active.update(frameDt);
            }
            Quaternion displayed = active.attitude();
            displayed.toMatrixColumnMajor(matrix);
            hud.record(displayed, active.setpointEulerDeg()); // feed the time-series history every frame

            float[] off = active.renderOffset();
            boolean firstPerson = ps != null && ps.firstPerson();
            if (firstPerson) {
                float hd = ps.headingRad(); // look forward along the drone heading
                engine.setCameraLookAt(off[0], off[1], off[2],
                        off[0] + (float) Math.cos(hd), off[1], off[2] - (float) Math.sin(hd));
            } else {
                engine.setCameraTarget(off[0], off[1], off[2]); // 3rd-person chase / orbit
            }

            engine.beginFrame();
            background.draw();
            if (controls.showGround) {
                ground.draw();
            }
            if (ps != null) {
                drawTrail(ps); // breadcrumb path in world space
            }

            if (!firstPerson) { // hide the body when the camera sits inside it
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
            }

            frames++;
            double fdt = (System.nanoTime() - fpsNanos) / 1e9;
            if (fdt >= 0.5) {
                fps = frames / fdt;
                frames = 0;
                fpsNanos = System.nanoTime();
            }

            if (controls.showHud) {
                if (ps != null) {
                    hud.drawPs5Hud(engine.fbWidth(), engine.fbHeight(), fps,
                            ps.name(), ps.bannerText(), ps.bannerColor(),
                            ps.firstPerson() ? "1st Person" : "3rd Person",
                            ps.trailActive(), ps.worldX(), ps.worldY(), ps.worldZ(), controls);
                } else {
                    hud.draw(engine.fbWidth(), engine.fbHeight(), state, displayed, fps,
                            active.name(), active.bannerText(), active.bannerColor(), active.simStatus(), simCtl, controls);
                }
            }

            engine.endFrame();
        }

        for (Mode m : modes) {
            m.dispose();
        }
        engine.terminate();
    }

    /** Renders the breadcrumb ring buffer as a line strip + point cloud in world space. */
    private static void drawTrail(PS5ControllerMode ps) {
        int n = ps.trailSize();
        if (n < 1) {
            return;
        }
        glDisable(GL_LIGHTING);
        glColor3f(0.35f, 0.85f, 1f);
        glLineWidth(2f);
        glBegin(GL_LINE_STRIP);
        for (int i = 0; i < n; i++) {
            glVertex3f(ps.trailGlX(i), ps.trailGlY(i), ps.trailGlZ(i));
        }
        glEnd();
        glPointSize(4f);
        glBegin(GL_POINTS);
        for (int i = 0; i < n; i++) {
            glVertex3f(ps.trailGlX(i), ps.trailGlY(i), ps.trailGlZ(i));
        }
        glEnd();
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
        System.out.println("Java: Tab switches mode (LIVE / SIM / PS5).");
        System.out.println("Java: global - H hud | 1 attitude | 2 panel | 3 graph | 4 hints | C drone/cube | G grid | Space pause | Esc quit");
        System.out.println("Java: LIVE   - R recenter | M mock/live");
        System.out.println("Java: SIM    - arrows tilt | Q/E yaw | T roll-step | Z gust | 0 level | R reset | [ ] select gain | - = adjust");
        System.out.println("Java: SIM    - mouse: drag gain sliders, click presets (Open Loop / P / PI / PD / Tuned / Oscillating / I Windup / Noisy D)");
        System.out.println("Java: PS5    - L-stick move | R2/L2 boost | D-pad up/down alt | Triangle trail | Square view");
        System.out.println("Java: PS5    - keyboard fallback: WASD move | Q/E down/up | V view | B trail | R recenter");
    }
}
