package com.kagenou.Avionics;

/**
 * Central, immutable app configuration so nothing (window size, port, baud,
 * smoothing) is hardcoded across the code. Built from CLI args in {@link #fromArgs}.
 *
 * <p>Supported args:
 * <pre>
 *   --demo            use the synthetic mock feed (no hardware needed)
 *   --port COM9       serial port to read the live IMU from
 *   --baud 115200     serial baud rate (default 115200)
 *   --slerp 0.35      attitude smoothing 0..1 (1 = no smoothing)
 *   --history 20      time-series graph window in seconds
 *   --width / --height window size in pixels
 * </pre>
 * {@code --demo} forces the synthetic feed. Otherwise the app auto-detects a
 * serial port (using {@code --port} if given) and falls back to the mock feed if
 * none is found, so it always runs out of the box.
 */
public record AppConfig(
        int width,
        int height,
        String title,
        String port,
        int baud,
        boolean demo,
        float slerpFactor,
        double historySeconds) {

    public static AppConfig fromArgs(String[] args) {
        int width = 1280;
        int height = 800;
        String title = "IMU Attitude Visualizer";
        String port = null;
        int baud = 115200;
        boolean demoFlag = false;
        float slerp = 0.35f;
        double history = 20.0;

        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "--demo" -> demoFlag = true;
                case "--port" -> port = next(args, ++i, "--port");
                case "--baud" -> baud = Integer.parseInt(next(args, ++i, "--baud"));
                case "--slerp" -> slerp = Float.parseFloat(next(args, ++i, "--slerp"));
                case "--history" -> history = Double.parseDouble(next(args, ++i, "--history"));
                case "--width" -> width = Integer.parseInt(next(args, ++i, "--width"));
                case "--height" -> height = Integer.parseInt(next(args, ++i, "--height"));
                default -> System.out.println("Java: ignoring unknown arg '" + args[i] + "'");
            }
        }

        // Only --demo forces mock; otherwise Main auto-detects a port (and still
        // falls back to mock if none is found).
        return new AppConfig(width, height, title, port, baud, demoFlag, clamp01(slerp), Math.max(2.0, history));
    }

    private static String next(String[] args, int i, String flag) {
        if (i >= args.length) {
            throw new IllegalArgumentException("missing value for " + flag);
        }
        return args[i];
    }

    private static float clamp01(float v) {
        return Math.max(0f, Math.min(1f, v));
    }
}
