package com.kagenou.avionics.hud;

import org.lwjgl.BufferUtils;
import org.lwjgl.stb.STBEasyFont;

import java.nio.ByteBuffer;

import static org.lwjgl.opengl.GL11.*;

/**
 * Tiny bitmap-font text via {@code STBEasyFont} (from the already-present lwjgl-stb
 * dependency, so no new deps). Each call fills a reused vertex buffer with quads
 * and draws them in the current (ortho) projection. Color comes from {@code glColor}
 * rather than the per-vertex color stb emits.
 */
public final class TextRenderer {
    // 64 KiB holds well over a thousand quads — far more than any single HUD line.
    private final ByteBuffer quads = BufferUtils.createByteBuffer(64 * 1024);

    /** Native font cell height in pixels (before scaling). */
    public static final float CELL_HEIGHT = 7f;

    public void draw(float x, float y, float scale, float[] rgb, float alpha, CharSequence text) {
        quads.clear();
        int n = STBEasyFont.stb_easy_font_print(0, 0, text, null, quads);

        glPushMatrix();
        glTranslatef(x, y, 0);
        glScalef(scale, scale, 1);
        glColor4f(rgb[0], rgb[1], rgb[2], alpha);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(2, GL_FLOAT, 16, quads); // stride 16: x,y,z float + 4 color bytes
        glDrawArrays(GL_QUADS, 0, n * 4);
        glDisableClientState(GL_VERTEX_ARRAY);
        glPopMatrix();
    }

    public float width(CharSequence text, float scale) {
        return STBEasyFont.stb_easy_font_width(text) * scale;
    }
}
