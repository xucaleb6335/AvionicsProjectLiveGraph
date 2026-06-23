package com.kagenou.avionics.io;

/**
 * Parses one line of the IMU wire format into a quaternion.
 *
 * <p>Canonical format is {@code "%.4f %.4f %.4f %.4f\n"} = {@code w x y z}
 * (whitespace-separated floats), matching the Arduino sketch and the planned
 * STM32 firmware. The parser is deliberately tolerant: it ignores blank/garbage
 * lines and any leading {@code key:} prefixes (so an older {@code "w:.. x:.."}
 * stream still works), and simply takes the first four parseable floats. It
 * never throws — callers decide how to count failures.
 */
public final class QuatParser {
    private QuatParser() {}

    /**
     * @return {@code [w, x, y, z]} on success, or {@code null} if fewer than four
     *         floats could be read from the line.
     */
    public static float[] parse(String line) {
        if (line == null) {
            return null;
        }
        String[] tokens = line.trim().split("\\s+");
        float[] out = new float[4];
        int found = 0;
        for (String tok : tokens) {
            // Tolerate "w:0.1" style tokens by keeping only the part after ':'.
            int colon = tok.indexOf(':');
            if (colon >= 0) {
                tok = tok.substring(colon + 1);
            }
            if (tok.isEmpty()) {
                continue;
            }
            try {
                out[found] = Float.parseFloat(tok);
                if (++found == 4) {
                    return out;
                }
            } catch (NumberFormatException ignored) {
                // skip non-numeric tokens (labels, "Received:", etc.)
            }
        }
        return null;
    }
}
