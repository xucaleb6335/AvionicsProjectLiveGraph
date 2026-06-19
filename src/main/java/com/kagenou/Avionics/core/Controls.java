package com.kagenou.avionics.core;

/**
 * Mutable UI/interaction state shared between the GLFW key callback and the render
 * loop. Both run on the main thread (callbacks fire during {@code glfwPollEvents}),
 * so plain fields are fine. Action requests (recenter, source toggle) are latched
 * here and consumed by the loop.
 */
public final class Controls {
    public boolean showHud = true;
    public boolean showAttitude = true;
    public boolean showDiagnostics = true;
    public boolean showGraph = true;
    public boolean showHints = true;

    public boolean showDrone = true;   // false -> legacy cube
    public boolean showGround = true;
    public boolean paused = false;     // freeze the displayed attitude

    public boolean recenterRequested = false;     // zero attitude to the current pose
    public boolean toggleSourceRequested = false; // switch mock <-> live

    // Mouse state in HUD/ortho pixel coords (for sliders + preset buttons).
    public float mouseX, mouseY;
    public boolean mouseDown;          // left button currently held
    public boolean mousePressedEdge;   // left button went down this frame
}
