package com.kagenou.avionics.hud;

import static org.lwjgl.opengl.GL11.*;

/**
 * A semi-transparent rectangular HUD panel with an optional border, drawn in
 * ortho/screen space. (Square corners for now; rounded corners would be a cheap
 * later polish.)
 */
public final class Panel {
    private Panel() {}

    public static void draw(float x, float y, float w, float h,
                            float[] fill, float fillAlpha,
                            float[] border, float borderAlpha) {
        glColor4f(fill[0], fill[1], fill[2], fillAlpha);
        glBegin(GL_QUADS);
        glVertex2f(x, y);
        glVertex2f(x + w, y);
        glVertex2f(x + w, y + h);
        glVertex2f(x, y + h);
        glEnd();

        if (border != null) {
            glColor4f(border[0], border[1], border[2], borderAlpha);
            glBegin(GL_LINE_LOOP);
            glVertex2f(x + 0.5f, y + 0.5f);
            glVertex2f(x + w - 0.5f, y + 0.5f);
            glVertex2f(x + w - 0.5f, y + h - 0.5f);
            glVertex2f(x + 0.5f, y + h - 0.5f);
            glEnd();
        }
    }
}
