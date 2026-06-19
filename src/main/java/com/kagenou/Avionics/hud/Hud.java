package com.kagenou.avionics.hud;

import com.kagenou.avionics.core.Controls;
import com.kagenou.avionics.io.AttitudeState;
import com.kagenou.avionics.math.Quaternion;
import com.kagenou.avionics.scene.Palette;
import com.kagenou.avionics.sim.SimStatus;
import com.kagenou.avionics.sim.Simulator;

import static org.lwjgl.opengl.GL11.*;

/**
 * The 2D dashboard overlay, drawn in an ortho pass on top of the 3D scene. Mirrors
 * the reference GPS dashboard's feel: a color-coded status banner, metric panels
 * for attitude and feed diagnostics, and a control-hints bar. Individual panels
 * toggle via keys (the overlay analog of the dashboard's tabs).
 */
public final class Hud {
    private final TextRenderer text = new TextRenderer();
    private final TimeSeriesGraph graph = new TimeSeriesGraph();
    private int draggingGain = -1; // gain slider currently being dragged, or -1

    // Rolling link-rate estimate (samples/sec) from the parse-OK counter.
    private long lastRateNanos = System.nanoTime();
    private int lastParseOk = 0;
    private float linkHz = 0f;

    // Time-series history for the roll/pitch/yaw traces (actual) + setpoint overlay.
    private final double windowSec;
    private final RingBuffer[] channels;     // actual roll/pitch/yaw
    private final RingBuffer[] setChannels;  // commanded roll/pitch/yaw
    private static final float[][] TRACE_COLORS = {Palette.TRACE_ROLL, Palette.TRACE_PITCH, Palette.TRACE_YAW};
    private static final String[] TRACE_LABELS = {"roll", "pitch", "yaw"};

    public Hud(double windowSeconds) {
        this.windowSec = windowSeconds;
        int cap = Math.max(1024, (int) (windowSeconds * 120)); // headroom above typical fps
        channels = new RingBuffer[]{new RingBuffer(cap), new RingBuffer(cap), new RingBuffer(cap)};
        setChannels = new RingBuffer[]{new RingBuffer(cap), new RingBuffer(cap), new RingBuffer(cap)};
    }

    /**
     * Appends the current attitude (and, if present, the commanded setpoint) to the
     * trace history. Call every frame, independent of HUD visibility.
     */
    public void record(Quaternion displayed, float[] setpointDeg) {
        float[] e = displayed.toEulerDeg();
        long t = System.nanoTime();
        for (int i = 0; i < 3; i++) {
            channels[i].push(t, e[i]);
        }
        if (setpointDeg != null) {
            for (int i = 0; i < 3; i++) {
                setChannels[i].push(t, setpointDeg[i]);
            }
        }
    }

    public void draw(int w, int h, AttitudeState state, Quaternion displayed, double fps,
                     String modeName, String bannerText, float[] bannerColor,
                     SimStatus sim, Simulator simCtl, Controls c) {
        if (!c.mouseDown) {
            draggingGain = -1; // release ends any slider drag
        }
        updateLinkRate(state);
        beginOrtho(w, h);

        drawStatusBanner(w, modeName, bannerText, bannerColor, c.paused);
        float topY = 46;
        if (c.showAttitude) {
            topY = drawAttitudePanel(12, topY, displayed) + 10;
        }
        if (c.showDiagnostics) {
            if (sim != null) {
                drawSimPanel(12, topY, sim, simCtl, c);
            } else {
                drawDiagnosticsPanel(12, topY, state, fps);
            }
        }
        if (sim != null) {
            drawPresetsPanel(w, sim, simCtl, c);
        }
        if (c.showGraph) {
            float gh = 180, bottomMargin = c.showHints ? 32 : 8;
            RingBuffer[] setpoints = sim != null ? setChannels : null; // overlay only when a setpoint exists
            graph.draw(text, 12, h - bottomMargin - gh, w - 24, gh,
                    System.nanoTime(), windowSec, channels, setpoints, TRACE_COLORS, TRACE_LABELS);
        }
        if (c.showHints) {
            drawHintsBar(w, h);
        }

        endOrtho();
    }

    private void drawStatusBanner(int w, String modeName, String bannerText, float[] color, boolean paused) {
        float bh = 34;
        Panel.draw(0, 0, w, bh, Palette.PANEL_BG, 0.88f, null, 0);

        // Status indicator square.
        float d = 12, sx = 12, sy = bh / 2 - d / 2;
        glColor4f(color[0], color[1], color[2], 1f);
        glBegin(GL_QUADS);
        glVertex2f(sx, sy);
        glVertex2f(sx + d, sy);
        glVertex2f(sx + d, sy + d);
        glVertex2f(sx, sy + d);
        glEnd();

        String line = "[" + modeName + "]   " + bannerText + (paused ? "    |    PAUSED" : "");
        text.draw(36, 11, 2.0f, Palette.TEXT, 1f, line);
    }

    /** @return the y just below the panel. */
    private float drawAttitudePanel(float x, float y, Quaternion q) {
        float w = 252, lh = 20, s = 1.6f, pad = 12;
        float h = pad + 7 * lh;
        Panel.draw(x, y, w, h, Palette.PANEL_BG, 0.8f, Palette.PANEL_BORDER, 0.55f);

        float[] e = q.toEulerDeg();
        float tx = x + pad, ty = y + pad;
        text.draw(tx, ty, s, Palette.TEXT_DIM, 1f, "QUATERNION");
        ty += lh;
        text.draw(tx, ty, s, Palette.TEXT, 1f, String.format("w %+.4f   x %+.4f", q.w, q.x));
        ty += lh;
        text.draw(tx, ty, s, Palette.TEXT, 1f, String.format("y %+.4f   z %+.4f", q.y, q.z));
        ty += lh + 6;
        text.draw(tx, ty, s, Palette.TEXT_DIM, 1f, "ATTITUDE (deg)");
        ty += lh;
        text.draw(tx, ty, s, Palette.TEXT, 1f, String.format("roll  %+7.1f", e[0]));
        ty += lh;
        text.draw(tx, ty, s, Palette.TEXT, 1f, String.format("pitch %+7.1f", e[1]));
        ty += lh;
        text.draw(tx, ty, s, Palette.TEXT, 1f, String.format("yaw   %+7.1f", e[2]));

        return y + h;
    }

    private void drawDiagnosticsPanel(float x, float y, AttitudeState state, double fps) {
        float w = 252, lh = 20, s = 1.6f, pad = 12;
        float h = pad + 5 * lh;
        Panel.draw(x, y, w, h, Palette.PANEL_BG, 0.8f, Palette.PANEL_BORDER, 0.55f);

        long age = state.ageMillis();
        float tx = x + pad, ty = y + pad;
        text.draw(tx, ty, s, Palette.TEXT_DIM, 1f, "DIAGNOSTICS");
        ty += lh;
        text.draw(tx, ty, s, Palette.TEXT, 1f, String.format("link    %.1f Hz", linkHz));
        ty += lh;
        text.draw(tx, ty, s, Palette.TEXT, 1f, "age     " + (age < 0 ? "--" : age + " ms"));
        ty += lh;
        text.draw(tx, ty, s, Palette.TEXT, 1f, String.format("lines   %d   err %d", state.totalLines(), state.parseErr()));
        ty += lh;
        text.draw(tx, ty, s, Palette.TEXT, 1f, String.format("render  %.0f fps", fps));
    }

    private void drawSimPanel(float x, float y, SimStatus s, Simulator simCtl, Controls c) {
        float w = 256, pad = 12, lh = 16, sVal = 1.45f, h = 360;
        Panel.draw(x, y, w, h, Palette.PANEL_BG, 0.82f, Palette.PANEL_BORDER, 0.55f);

        float tx = x + pad, ty = y + pad;
        text.draw(tx, ty, 1.55f, Palette.TEXT_DIM, 1f, "MOTORS");
        ty += lh + 2;

        // Motor output bars, colored by load.
        for (int i = 0; i < 4; i++) {
            float frac = clamp01(s.motors()[i]);
            text.draw(tx, ty, sVal, Palette.TEXT, 1f, "M" + (i + 1));
            float bx = tx + 30, bw = 150, bh = 9;
            Panel.draw(bx, ty, bw, bh, Palette.PANEL_BORDER, 0.25f, null, 0);
            float[] col = frac > 0.9f ? Palette.ACCENT_RED : frac > 0.7f ? Palette.ACCENT_AMBER : Palette.ACCENT_GREEN;
            glColor4f(col[0], col[1], col[2], 0.9f);
            fillRect(bx, ty, bw * frac, bh);
            text.draw(bx + bw + 8, ty, 1.25f, Palette.TEXT_DIM, 1f, String.format("%.2f", frac));
            ty += 16;
        }
        ty += 6;

        float[] sp = s.setpointDeg(), ac = s.attitudeDeg();
        text.draw(tx, ty, sVal, Palette.TEXT_DIM, 1f, String.format("set  r%+5.0f p%+5.0f y%+5.0f", sp[0], sp[1], sp[2]));
        ty += lh;
        text.draw(tx, ty, sVal, Palette.TEXT, 1f, String.format("act  r%+5.1f p%+5.1f y%+5.1f", ac[0], ac[1], ac[2]));
        ty += lh;
        text.draw(tx, ty, sVal, Palette.TEXT, 1f, String.format("alt  %.2f / %.2f m", s.altitude(), s.altSetpoint()));
        ty += lh + 4;

        text.draw(tx, ty, 1.2f, Palette.TEXT_DIM, 1f, "GAINS  (drag sliders)");
        ty += 16;
        for (int i = 0; i < s.gainNames().length; i++) {
            ty = drawGainSlider(i, tx, ty, s, simCtl, c);
        }
        ty += 2;

        text.draw(tx, ty, 1.2f, Palette.TEXT_DIM, 1f, "STEP (T)");
        if (s.stepActive()) {
            String rise = Float.isNaN(s.stepRiseMs()) ? "--" : String.format("%.0fms", s.stepRiseMs());
            text.draw(tx + 72, ty, 1.3f, Palette.TEXT, 1f,
                    String.format("rise %s OS %.1f%% ts %.2fs", rise, s.stepOvershootPct(), s.stepSettleS()));
        } else {
            text.draw(tx + 72, ty, 1.3f, Palette.TEXT_DIM, 1f, "press T");
        }
        ty += 16;
        text.draw(tx, ty, 1.05f, Palette.TEXT_DIM, 1f, "arrows tilt  W/S alt  Q/E yaw  T step  Z gust");
    }

    /** Draws one gain as a mouse-draggable slider and returns the next y. */
    private float drawGainSlider(int i, float x, float y, SimStatus s, Simulator simCtl, Controls c) {
        float rowH = 20;
        float gmin = s.gainMin()[i], gmax = s.gainMax()[i];
        float val = simCtl != null ? simCtl.gainValue(i) : s.gainValues()[i];

        float trackX = x + 56, trackW = 104, trackY = y + 7, trackH = 4;

        boolean over = c.mouseX >= trackX - 7 && c.mouseX <= trackX + trackW + 7
                && c.mouseY >= y - 1 && c.mouseY <= y + rowH - 2;
        if (simCtl != null) {
            if (c.mousePressedEdge && over) {
                draggingGain = i;
            }
            if (draggingGain == i && c.mouseDown) {
                float f = clamp01((c.mouseX - trackX) / trackW);
                simCtl.setGain(i, gmin + f * (gmax - gmin));
                val = simCtl.gainValue(i);
            }
        }
        float frac = gmax > gmin ? clamp01((val - gmin) / (gmax - gmin)) : 0f;
        boolean active = i == s.selectedGain() || draggingGain == i;
        float[] accent = active ? Palette.ACCENT_GREEN : Palette.TEXT;

        text.draw(x, y + 2, 1.2f, active ? Palette.ACCENT_GREEN : Palette.TEXT_DIM, 1f, s.gainNames()[i]);
        glColor4f(Palette.PANEL_BORDER[0], Palette.PANEL_BORDER[1], Palette.PANEL_BORDER[2], 0.5f);
        fillRect(trackX, trackY, trackW, trackH);
        glColor4f(accent[0], accent[1], accent[2], 0.55f);
        fillRect(trackX, trackY, trackW * frac, trackH);
        float handleX = trackX + trackW * frac;
        glColor4f(accent[0], accent[1], accent[2], 1f);
        fillRect(handleX - 3, y + 1, 6, rowH - 4);
        text.draw(trackX + trackW + 10, y + 2, 1.1f, Palette.TEXT, 1f, String.format("%.4f", val));

        return y + rowH;
    }

    private void drawPresetsPanel(int w, SimStatus s, Simulator simCtl, Controls c) {
        String[] names = s.presetNames();
        float pad = 10, pw = 168, x = w - pw - 12, y = 46, rowH = 26, headerH = 24;
        float h = headerH + names.length * rowH + 8;
        Panel.draw(x, y, pw, h, Palette.PANEL_BG, 0.82f, Palette.PANEL_BORDER, 0.55f);
        text.draw(x + pad, y + 8, 1.4f, Palette.TEXT_DIM, 1f, "PRESETS  (click)");

        float by = y + headerH;
        for (int i = 0; i < names.length; i++) {
            float bx = x + pad, bw = pw - 2 * pad, bh = rowH - 5;
            boolean sel = i == s.currentPreset();
            boolean over = c.mouseX >= bx && c.mouseX <= bx + bw && c.mouseY >= by && c.mouseY <= by + bh;
            float[] fill = sel ? Palette.ACCENT_GREEN : Palette.PANEL_BORDER;
            float fa = sel ? 0.28f : (over ? 0.35f : 0.12f);
            float[] border = sel ? Palette.ACCENT_GREEN : Palette.PANEL_BORDER;
            Panel.draw(bx, by, bw, bh, fill, fa, border, sel ? 0.9f : 0.4f);
            text.draw(bx + 8, by + 5, 1.3f, sel ? Palette.ACCENT_GREEN : Palette.TEXT, 1f, names[i]);
            if (simCtl != null && c.mousePressedEdge && over) {
                simCtl.applyPreset(i);
            }
            by += rowH;
        }
    }

    private static void fillRect(float x, float y, float w, float h) {
        glBegin(GL_QUADS);
        glVertex2f(x, y);
        glVertex2f(x + w, y);
        glVertex2f(x + w, y + h);
        glVertex2f(x, y + h);
        glEnd();
    }

    private static float clamp01(float v) {
        return v < 0 ? 0 : Math.min(v, 1f);
    }

    private void drawHintsBar(int w, int h) {
        float bh = 24, y = h - bh;
        Panel.draw(0, y, w, bh, Palette.PANEL_BG, 0.8f, null, 0);
        text.draw(12, y + 7, 1.25f, Palette.TEXT_DIM, 1f,
                "Tab mode   H hud   1 attitude   2 panel   3 graph   4 hints   C drone/cube   G grid   Space pause   Esc quit");
    }

    private void updateLinkRate(AttitudeState state) {
        long now = System.nanoTime();
        double dt = (now - lastRateNanos) / 1e9;
        if (dt >= 0.5) {
            int ok = state.parseOk();
            linkHz = (float) ((ok - lastParseOk) / dt);
            lastParseOk = ok;
            lastRateNanos = now;
        }
    }

    private void beginOrtho(int w, int h) {
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, w, h, 0, -1, 1); // top-left origin, y down
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    private void endOrtho() {
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
    }
}
