package com.kagenou.Avionics.scene;

/**
 * Cohesive dark-theme palette for the scene, matching the look of the reference
 * GPS dashboard. Colors are {@code {r, g, b}} in 0..1. The status accents are not
 * used until the HUD lands (M4) but are defined here so the whole UI shares one
 * source of truth.
 */
public final class Palette {
    private Palette() {}

    // Vertical background gradient (top is darker).
    public static final float[] BG_TOP    = {0.04f, 0.06f, 0.11f};
    public static final float[] BG_BOTTOM = {0.12f, 0.15f, 0.22f};

    // Ground.
    public static final float[] GRID   = {0.30f, 0.34f, 0.42f};
    public static final float[] AXIS_X = {0.82f, 0.32f, 0.32f};
    public static final float[] AXIS_Z = {0.36f, 0.56f, 0.86f};
    public static final float[] SHADOW = {0.00f, 0.00f, 0.00f};

    // HUD panels + text.
    public static final float[] PANEL_BG     = {0.10f, 0.12f, 0.17f};
    public static final float[] PANEL_BORDER = {0.30f, 0.34f, 0.42f};
    public static final float[] TEXT         = {0.90f, 0.92f, 0.96f};
    public static final float[] TEXT_DIM     = {0.58f, 0.62f, 0.70f};

    // Status accents for the HUD.
    public static final float[] ACCENT_GREEN = {0.30f, 0.80f, 0.45f};
    public static final float[] ACCENT_AMBER = {0.95f, 0.70f, 0.20f};
    public static final float[] ACCENT_RED   = {0.90f, 0.25f, 0.25f};

    // Time-series traces (roll / pitch / yaw).
    public static final float[] TRACE_ROLL  = {0.92f, 0.48f, 0.42f};
    public static final float[] TRACE_PITCH = {0.45f, 0.85f, 0.55f};
    public static final float[] TRACE_YAW   = {0.45f, 0.66f, 0.95f};
}
