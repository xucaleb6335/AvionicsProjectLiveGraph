package com.kagenou.Avionics.scene;

import static org.lwjgl.opengl.GL11.*;

/**
 * The ground reference: a grid whose lines fade with distance from the origin,
 * subtle colored origin axes, and a soft drop-shadow blob under the drone for
 * depth. Replaces the old {@code Cube.drawGrid}. Drawn unlit with alpha blending;
 * the shadow is drawn last so it blends over the grid.
 */
public final class Ground {
    private static final float Y = -0.6f;     // ground plane height
    private static final int SIZE = 6;        // grid half-extent (world units)
    private static final float STEP = 0.5f;
    private static final float GRID_ALPHA = 0.6f;

    public void draw() {
        boolean blendWasOff = !glIsEnabled(GL_BLEND);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_LIGHTING);

        drawGrid();
        drawAxes();
        drawShadow(); // last: blends over the grid

        if (blendWasOff) {
            glDisable(GL_BLEND);
        }
    }

    private static final int FADE_SEGMENTS = 24; // per line, so the fade can vary along it

    private void drawGrid() {
        float[] c = Palette.GRID;
        glLineWidth(1f);
        for (float x = -SIZE; x <= SIZE + 1e-3f; x += STEP) {
            fadingLine(c, x, -SIZE, x, SIZE);
        }
        for (float z = -SIZE; z <= SIZE + 1e-3f; z += STEP) {
            fadingLine(c, -SIZE, z, SIZE, z);
        }
    }

    /**
     * Draws one grid line as a subdivided strip whose per-vertex alpha fades
     * radially from the origin. Subdivision matters: a line's endpoints are both
     * at the grid edge (alpha ~0), so without interior vertices the whole line
     * would interpolate to invisible.
     */
    private void fadingLine(float[] c, float x1, float z1, float x2, float z2) {
        glBegin(GL_LINE_STRIP);
        for (int i = 0; i <= FADE_SEGMENTS; i++) {
            float t = (float) i / FADE_SEGMENTS;
            float x = x1 + (x2 - x1) * t;
            float z = z1 + (z2 - z1) * t;
            float d = (float) Math.sqrt(x * x + z * z);
            float a = Math.max(0f, 1f - d / SIZE) * GRID_ALPHA;
            glColor4f(c[0], c[1], c[2], a);
            glVertex3f(x, Y, z);
        }
        glEnd();
    }

    private void drawAxes() {
        float yy = Y + 0.002f; // just above the grid to avoid z-fighting
        float len = 1.2f;
        glLineWidth(2f);
        glBegin(GL_LINES);
        float[] xc = Palette.AXIS_X;
        glColor4f(xc[0], xc[1], xc[2], 0.9f);
        glVertex3f(-len, yy, 0);
        glVertex3f(len, yy, 0);
        float[] zc = Palette.AXIS_Z;
        glColor4f(zc[0], zc[1], zc[2], 0.9f);
        glVertex3f(0, yy, -len);
        glVertex3f(0, yy, len);
        glEnd();
        glLineWidth(1f);
    }

    private void drawShadow() {
        float[] s = Palette.SHADOW;
        glPushMatrix();
        glTranslatef(0, Y + 0.001f, 0);
        glColor4f(s[0], s[1], s[2], 0.30f);
        Disc.draw(0.55f, 40);
        glPopMatrix();
    }
}
