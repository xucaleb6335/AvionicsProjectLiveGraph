package com.kagenou.Avionics.scene;

import static org.lwjgl.opengl.GL11.*;

/**
 * Immediate-mode capped cylinder aligned to the Y axis, centered at the origin,
 * drawn in the current GL color with radial/axial normals for lighting. Used for
 * the {@link Drone}'s motors and prop hubs.
 */
public final class Cylinder {
    private Cylinder() {}

    public static void draw(float radius, float height, int segments) {
        float y1 = height * 0.5f, y0 = -height * 0.5f;

        // Side wall.
        glBegin(GL_QUAD_STRIP);
        for (int i = 0; i <= segments; i++) {
            double a = 2 * Math.PI * i / segments;
            float cx = (float) Math.cos(a), cz = (float) Math.sin(a);
            glNormal3f(cx, 0, cz);
            glVertex3f(cx * radius, y1, cz * radius);
            glVertex3f(cx * radius, y0, cz * radius);
        }
        glEnd();

        // Top cap.
        glBegin(GL_TRIANGLE_FAN);
        glNormal3f(0, 1, 0);
        glVertex3f(0, y1, 0);
        for (int i = 0; i <= segments; i++) {
            double a = 2 * Math.PI * i / segments;
            glVertex3f((float) Math.cos(a) * radius, y1, (float) Math.sin(a) * radius);
        }
        glEnd();

        // Bottom cap (reverse winding so the normal faces down).
        glBegin(GL_TRIANGLE_FAN);
        glNormal3f(0, -1, 0);
        glVertex3f(0, y0, 0);
        for (int i = segments; i >= 0; i--) {
            double a = 2 * Math.PI * i / segments;
            glVertex3f((float) Math.cos(a) * radius, y0, (float) Math.sin(a) * radius);
        }
        glEnd();
    }
}
