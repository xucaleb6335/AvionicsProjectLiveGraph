package com.kagenou.avionics.scene;

import static org.lwjgl.opengl.GL11.*;

/**
 * Immediate-mode axis-aligned box centered at the origin, drawn in the current
 * GL color. Per-face normals are emitted so the shape reads correctly once
 * lighting is enabled. A reusable building block for {@link Drone}.
 */
public final class Box {
    private Box() {}

    /** Draws a box of full dimensions {@code sx × sy × sz} centered at the origin. */
    public static void draw(float sx, float sy, float sz) {
        float x = sx * 0.5f, y = sy * 0.5f, z = sz * 0.5f;
        glBegin(GL_QUADS);

        glNormal3f(0, 0, 1); // front (+Z)
        glVertex3f(-x, -y, z); glVertex3f(x, -y, z); glVertex3f(x, y, z); glVertex3f(-x, y, z);

        glNormal3f(0, 0, -1); // back (-Z)
        glVertex3f(-x, -y, -z); glVertex3f(-x, y, -z); glVertex3f(x, y, -z); glVertex3f(x, -y, -z);

        glNormal3f(-1, 0, 0); // left (-X)
        glVertex3f(-x, -y, -z); glVertex3f(-x, -y, z); glVertex3f(-x, y, z); glVertex3f(-x, y, -z);

        glNormal3f(1, 0, 0); // right (+X)
        glVertex3f(x, -y, -z); glVertex3f(x, y, -z); glVertex3f(x, y, z); glVertex3f(x, -y, z);

        glNormal3f(0, 1, 0); // top (+Y)
        glVertex3f(-x, y, -z); glVertex3f(-x, y, z); glVertex3f(x, y, z); glVertex3f(x, y, -z);

        glNormal3f(0, -1, 0); // bottom (-Y)
        glVertex3f(-x, -y, -z); glVertex3f(x, -y, -z); glVertex3f(x, -y, z); glVertex3f(-x, -y, z);

        glEnd();
    }
}
