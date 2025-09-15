package com.kagenou.Avionics;

import org.lwjgl.opengl.GL;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL13C.GL_MULTISAMPLE;
import static org.lwjgl.system.MemoryUtil.*;

/**
 * Owns the GLFW window and OpenGL context and drives the per-frame projection /
 * camera setup. Rendering of scene content (grid, cube/drone) is done by the
 * caller between {@link #beginFrame()} and {@link #endFrame()}.
 *
 * <p>Reworked from the original: window size comes from {@link AppConfig}, the
 * framebuffer-size callback keeps the viewport and projection aspect correct on
 * resize, and the attitude/quaternion math has moved out to {@code math} and the
 * render loop. The old stray {@code glRotatef} and discarded rotation matrix are
 * gone.
 */
public final class  Engine {
    private final long window;

    // Framebuffer size in pixels; updated by the resize callback. May differ from
    // window size on HiDPI displays, so we drive viewport/aspect from this.
    private int fbWidth;
    private int fbHeight;

    // Orbit camera: looks at a target (default origin) from a fixed relative offset,
    // so it can follow a moving drone.
    private static final float[] EYE_OFFSET = {0f, 1.2f, 3.5f};
    private static final float[] UP = {0f, 1f, 0f};
    private float ctx, cty, ctz; // camera target

    // Directional lights (w == 0), in world space: a key from upper-front-right and
    // a dim cool fill from the opposite side.
    private static final float[] LIGHT_DIR = {0.4f, 1.0f, 0.7f, 0f};
    private static final float[] FILL_DIR = {-0.5f, 0.4f, -0.6f, 0f};

    public Engine(AppConfig cfg) {
        if (!glfwInit()) {
            throw new IllegalStateException("Unable to initialize GLFW");
        }

        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
        glfwWindowHint(GLFW_SAMPLES, 4); // request MSAA to match GL_MULTISAMPLE

        window = glfwCreateWindow(cfg.width(), cfg.height(), cfg.title(), NULL, NULL);
        if (window == NULL) {
            throw new RuntimeException("Failed to create GLFW window");
        }

        glfwMakeContextCurrent(window);
        glfwSwapInterval(1); // vsync
        GL.createCapabilities();

        glEnable(GL_MULTISAMPLE);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glClearColor(0.08f, 0.09f, 0.12f, 1.0f);

        // Lighting so solid shapes read as 3D. glColorMaterial lets primitives keep
        // using glColor3f for their per-part colors. GL_LIGHTING itself stays off
        // here and is toggled per-object by the render loop (lit drone vs. flat
        // grid/cube). A warm key light models the form; a dim cool fill light from
        // the opposite side keeps the shadowed faces from going black and ties them
        // to the gradient backdrop. Subtle specular adds a little sheen.
        glEnable(GL_NORMALIZE);
        glEnable(GL_COLOR_MATERIAL);
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

        glLightfv(GL_LIGHT0, GL_AMBIENT, new float[]{0.22f, 0.22f, 0.25f, 1f});
        glLightfv(GL_LIGHT0, GL_DIFFUSE, new float[]{0.90f, 0.90f, 0.86f, 1f});
        glLightfv(GL_LIGHT0, GL_SPECULAR, new float[]{0.60f, 0.60f, 0.60f, 1f});
        glEnable(GL_LIGHT0);

        glLightfv(GL_LIGHT1, GL_DIFFUSE, new float[]{0.16f, 0.19f, 0.26f, 1f}); // cool fill
        glEnable(GL_LIGHT1);

        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, new float[]{0.22f, 0.22f, 0.24f, 1f});
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 18f);

        // Initialize framebuffer size and keep it current on resize.
        int[] w = new int[1], h = new int[1];
        glfwGetFramebufferSize(window, w, h);
        fbWidth = w[0];
        fbHeight = h[0];
        glfwSetFramebufferSizeCallback(window, (win, newW, newH) -> {
            fbWidth = newW;
            fbHeight = newH;
        });
    }

    public long window() {
        return window;
    }

    /** Point the camera at this world position (it keeps a fixed relative offset). */
    public void setCameraTarget(float x, float y, float z) {
        ctx = x;
        cty = y;
        ctz = z;
    }

    public int fbWidth() {
        return fbWidth;
    }

    public int fbHeight() {
        return fbHeight;
    }

    /** Clears the frame and sets up projection + camera. Call once per frame first. */
    public void beginFrame() {
        glViewport(0, 0, fbWidth, fbHeight);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float aspect = fbHeight == 0 ? 1f : (float) fbWidth / fbHeight;
        perspective(60f, aspect, 0.1f, 100f);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        float[] center = {ctx, cty, ctz};
        float[] eye = {ctx + EYE_OFFSET[0], cty + EYE_OFFSET[1], ctz + EYE_OFFSET[2]};
        lookAt(eye, center, UP);

        // Set light positions under the view transform so they stay fixed in world space.
        glLightfv(GL_LIGHT0, GL_POSITION, LIGHT_DIR);
        glLightfv(GL_LIGHT1, GL_POSITION, FILL_DIR);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    /** Presents the frame and pumps window events. Call once per frame last. */
    public void endFrame() {
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    public void terminate() {
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    // --- fixed-function helpers (no GLU dependency) ---

    private static void perspective(float fovYDeg, float aspect, float zNear, float zFar) {
        float fH = (float) Math.tan(Math.toRadians(fovYDeg / 2)) * zNear;
        float fW = fH * aspect;
        glFrustum(-fW, fW, -fH, fH, zNear, zFar);
    }

    private static void lookAt(float[] eye, float[] center, float[] up) {
        float[] f = normalize(sub(center, eye));
        float[] s = normalize(cross(f, up));
        float[] u = cross(s, f);

        // Column-major view matrix.
        float[] m = {
                s[0], u[0], -f[0], 0,
                s[1], u[1], -f[1], 0,
                s[2], u[2], -f[2], 0,
                0, 0, 0, 1
        };
        glMultMatrixf(m);
        glTranslatef(-eye[0], -eye[1], -eye[2]);
    }

    private static float[] sub(float[] a, float[] b) {
        return new float[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static float[] cross(float[] a, float[] b) {
        return new float[]{
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
        };
    }

    private static float[] normalize(float[] v) {
        float n = (float) Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (n < 1e-8f) {
            return new float[]{0, 0, 0};
        }
        return new float[]{v[0] / n, v[1] / n, v[2] / n};
    }
}
