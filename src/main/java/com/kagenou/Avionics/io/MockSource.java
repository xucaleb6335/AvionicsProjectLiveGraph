package com.kagenou.Avionics.io;

import com.kagenou.Avionics.math.Quaternion;

/**
 * Synthetic attitude feed for developing and verifying the visualizer without
 * hardware (enabled with {@code --demo}). Emits a smooth compound rotation at a
 * steady rate so every downstream feature can be exercised on the laptop alone.
 */
public final class MockSource implements AttitudeSource {
    private static final int RATE_HZ = 100;

    private final AttitudeState state;
    private Thread thread;
    private volatile boolean running;

    public MockSource(AttitudeState state) {
        this.state = state;
    }

    @Override
    public void start() {
        running = true;
        thread = new Thread(this::loop, "mock-imu");
        thread.setDaemon(true);
        thread.start();
        System.out.println("Java: mock IMU feed started (" + RATE_HZ + " Hz)");
    }

    private void loop() {
        long start = System.nanoTime();
        long periodNanos = 1_000_000_000L / RATE_HZ;
        while (running) {
            float t = (System.nanoTime() - start) / 1e9f;
            // Gentle, readable motion: bounded roll/pitch wobble + slow continuous yaw.
            float roll = 0.6f * (float) Math.sin(t * 0.7f);
            float pitch = 0.5f * (float) Math.sin(t * 0.5f + 1.0f);
            float yaw = t * 0.3f;
            Quaternion q = Quaternion.fromEuler(roll, pitch, yaw);

            state.recordLine();
            state.update(q.w, q.x, q.y, q.z);

            try {
                Thread.sleep(periodNanos / 1_000_000L, (int) (periodNanos % 1_000_000L));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    @Override
    public void stop() {
        running = false;
        if (thread != null) {
            thread.interrupt();
        }
    }

    @Override
    public String describe() {
        return "mock IMU (synthetic)";
    }
}
