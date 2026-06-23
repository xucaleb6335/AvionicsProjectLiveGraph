package com.kagenou.avionics.scene;

import static org.lwjgl.opengl.GL11.*;

/**
 * Full-screen vertical gradient backdrop, drawn in screen space behind the 3D
 * scene (replaces the flat {@code glClearColor}). Renders a single quad in NDC
 * with depth test/write off so it never occludes the scene drawn afterward, then
 * restores the projection/modelview the caller set up.
 */
public final class Background {

    public void draw() {
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glDepthMask(false);

        float[] top = Palette.BG_TOP, bot = Palette.BG_BOTTOM;
        glBegin(GL_QUADS);
        glColor3f(bot[0], bot[1], bot[2]);
        glVertex2f(-1f, -1f);
        glVertex2f(1f, -1f);
        glColor3f(top[0], top[1], top[2]);
        glVertex2f(1f, 1f);
        glVertex2f(-1f, 1f);
        glEnd();

        glDepthMask(true);
        glEnable(GL_DEPTH_TEST);

        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
    }
}
