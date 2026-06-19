package com.kagenou.avionics.io;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Thread-safe hand-off of the latest attitude sample from a producer (serial or
 * mock thread) to the render thread.
 *
 * <p>The quaternion is published as an immutable {@code float[4]} snapshot via an
 * {@link AtomicReference}, so a reader always sees a self-consistent (w,x,y,z)
 * rather than a torn mix of an old and new sample. Counters back the future
 * diagnostics panel; they are wired now so M1 already tracks feed health.
 */
public final class AttitudeState {
    private final AtomicReference<float[]> latest =
            new AtomicReference<>(new float[]{1, 0, 0, 0}); // identity until first sample

    private volatile long lastUpdateNanos = 0L; // 0 == no sample yet

    private final AtomicInteger totalLines = new AtomicInteger();
    private final AtomicInteger parseOk = new AtomicInteger();
    private final AtomicInteger parseErr = new AtomicInteger();

    /** Producer: a line arrived (parsed or not). */
    public void recordLine() {
        totalLines.incrementAndGet();
    }

    /** Producer: publish a fresh, normalized-or-not quaternion snapshot. */
    public void update(float w, float x, float y, float z) {
        latest.set(new float[]{w, x, y, z});
        lastUpdateNanos = System.nanoTime();
        parseOk.incrementAndGet();
    }

    /** Producer: a line could not be parsed. */
    public void recordParseError() {
        parseErr.incrementAndGet();
    }

    /** Reader: latest snapshot. Treat as read-only — do not mutate the array. */
    public float[] snapshot() {
        return latest.get();
    }

    /** Milliseconds since the last sample, or {@code -1} if none has arrived. */
    public long ageMillis() {
        long t = lastUpdateNanos;
        return t == 0L ? -1 : (System.nanoTime() - t) / 1_000_000L;
    }

    public int totalLines() { return totalLines.get(); }
    public int parseOk()    { return parseOk.get(); }
    public int parseErr()   { return parseErr.get(); }
}
