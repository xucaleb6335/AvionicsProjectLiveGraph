package com.kagenou.avionics.hud;

import com.kagenou.avionics.scene.Palette;

import static org.lwjgl.opengl.GL11.*;

/**
 * Scrolling multi-channel line plot for the HUD: the newest sample sits at the
 * right edge and older samples scroll left over a fixed time window. Y auto-scales
 * to the data in view (with padding and a minimum span), with value gridlines, a
 * highlighted zero line, an axis box, and a legend. Each trace is decimated to at
 * most one point per horizontal pixel — the same idea as the reference dashboard's
 * row decimation.
 */
public final class TimeSeriesGraph {

    public void draw(TextRenderer text, float x, float y, float w, float h,
                     long now, double windowSec,
                     RingBuffer[] channels, RingBuffer[] setpoints, float[][] colors, String[] labels) {
        Panel.draw(x, y, w, h, Palette.PANEL_BG, 0.8f, Palette.PANEL_BORDER, 0.55f);

        float padL = 46, padR = 12, padT = 22, padB = 18;
        float px = x + padL, py = y + padT, pw = w - padL - padR, ph = h - padT - padB;
        long windowNanos = (long) (windowSec * 1e9);

        // --- Y auto-scale across all channels within the window ---
        float ymin = Float.MAX_VALUE, ymax = -Float.MAX_VALUE;
        float[] r1 = scan(channels, now, windowNanos, ymin, ymax);
        ymin = r1[0];
        ymax = r1[1];
        if (setpoints != null) {
            float[] r2 = scan(setpoints, now, windowNanos, ymin, ymax);
            ymin = r2[0];
            ymax = r2[1];
        }
        if (ymin > ymax) {                 // no data yet
            ymin = -10;
            ymax = 10;
        }
        if (ymax - ymin < 20) {            // keep a sane minimum span
            float c = (ymin + ymax) * 0.5f;
            ymin = c - 10;
            ymax = c + 10;
        }
        float padY = (ymax - ymin) * 0.1f;
        ymin -= padY;
        ymax += padY;
        float span = ymax - ymin;

        // --- gridlines + Y value labels ---
        glLineWidth(1f);
        for (int r = 0; r <= 4; r++) {
            float fy = py + ph * r / 4f;
            float val = ymax - span * r / 4f;
            glColor4f(Palette.GRID[0], Palette.GRID[1], Palette.GRID[2], 0.35f);
            glBegin(GL_LINES);
            glVertex2f(px, fy);
            glVertex2f(px + pw, fy);
            glEnd();
            text.draw(x + 6, fy - 4, 1.2f, Palette.TEXT_DIM, 1f, String.format("%4.0f", val));
        }
        if (ymin < 0 && ymax > 0) {        // highlighted zero line
            float fy = py + ph - (0 - ymin) / span * ph;
            glColor4f(Palette.TEXT_DIM[0], Palette.TEXT_DIM[1], Palette.TEXT_DIM[2], 0.7f);
            glBegin(GL_LINES);
            glVertex2f(px, fy);
            glVertex2f(px + pw, fy);
            glEnd();
        }
        glColor4f(Palette.PANEL_BORDER[0], Palette.PANEL_BORDER[1], Palette.PANEL_BORDER[2], 0.7f);
        glBegin(GL_LINE_LOOP);
        glVertex2f(px, py);
        glVertex2f(px + pw, py);
        glVertex2f(px + pw, py + ph);
        glVertex2f(px, py + ph);
        glEnd();

        // --- traces (faint setpoint overlay first, then bright actual) ---
        int maxPts = Math.max(2, (int) pw);
        if (setpoints != null) {
            glLineWidth(1f);
            for (int c = 0; c < setpoints.length; c++) {
                drawTrace(setpoints[c], colors[c], 0.30f, now, windowNanos, windowSec, px, py, pw, ph, ymin, span, maxPts);
            }
        }
        glLineWidth(1.5f);
        for (int c = 0; c < channels.length; c++) {
            drawTrace(channels[c], colors[c], 1f, now, windowNanos, windowSec, px, py, pw, ph, ymin, span, maxPts);
        }
        glLineWidth(1f);

        // --- title + legend ---
        text.draw(x + 8, y + 6, 1.3f, Palette.TEXT_DIM, 1f, String.format("ATTITUDE (deg)  -  last %.0fs", windowSec));
        drawLegend(text, x, y, w, channels.length, colors, labels);
    }

    /** Scans buffers for min/max within the window, seeded with running {@code lo}/{@code hi}. */
    private static float[] scan(RingBuffer[] bufs, long now, long windowNanos, float lo, float hi) {
        for (RingBuffer ch : bufs) {
            int n = ch.size();
            for (int i = 0; i < n; i++) {
                if (now - ch.timeAt(i) > windowNanos) {
                    continue;
                }
                float v = ch.valueAt(i);
                if (v < lo) lo = v;
                if (v > hi) hi = v;
            }
        }
        return new float[]{lo, hi};
    }

    private void drawTrace(RingBuffer ch, float[] color, float alpha, long now, long windowNanos, double windowSec,
                           float px, float py, float pw, float ph, float ymin, float span, int maxPts) {
        int n = ch.size();
        if (n < 2) {
            return;
        }
        int startI = n;
        for (int i = 0; i < n; i++) {
            if (now - ch.timeAt(i) <= windowNanos) {
                startI = i;
                break;
            }
        }
        int inWin = n - startI;
        if (inWin < 2) {
            return;
        }
        int stride = Math.max(1, inWin / maxPts);

        glColor4f(color[0], color[1], color[2], alpha);
        glBegin(GL_LINE_STRIP);
        for (int i = startI; i < n; i += stride) {
            float ageSec = (now - ch.timeAt(i)) / 1e9f;
            float fx = px + (1f - ageSec / (float) windowSec) * pw;
            float fy = py + ph - (ch.valueAt(i) - ymin) / span * ph;
            glVertex2f(fx, fy);
        }
        glEnd();
    }

    private void drawLegend(TextRenderer text, float x, float y, float w, int count,
                            float[][] colors, String[] labels) {
        float sq = 9, sqGap = 4, itemGap = 16, scale = 1.3f;
        float total = 0;
        for (int c = 0; c < count; c++) {
            total += sq + sqGap + text.width(labels[c], scale) + itemGap;
        }
        float cx = x + w - 12 - (total - itemGap);
        for (int c = 0; c < count; c++) {
            float[] col = colors[c];
            glColor4f(col[0], col[1], col[2], 1f);
            glBegin(GL_QUADS);
            glVertex2f(cx, y + 7);
            glVertex2f(cx + sq, y + 7);
            glVertex2f(cx + sq, y + 7 + sq);
            glVertex2f(cx, y + 7 + sq);
            glEnd();
            cx += sq + sqGap;
            text.draw(cx, y + 6, scale, col, 1f, labels[c]);
            cx += text.width(labels[c], scale) + itemGap;
        }
    }
}
