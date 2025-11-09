package com.kagenou.Avionics.scene;

import static org.lwjgl.opengl.GL11.*;

/**
 * Immediate-mode flat filled disc in the XZ plane (normal {@code +Y}), centered at
 * the origin, drawn in the current GL color. Used for the drone's ground shadow.
 */
public final class Disc {
    private Disc() {}

    public static void draw(float radius, int segments) {
        glBegin(GL_TRIANGLE_FAN);
        glNormal3f(0, 1, 0);
        glVertex3f(0, 0, 0);
        for (int i = 0; i <= segments; i++) {
            double a = 2 * Math.PI * i / segments;
            glVertex3f((float) Math.cos(a) * radius, 0, (float) Math.sin(a) * radius);
        }
        glEnd();
    }
}
