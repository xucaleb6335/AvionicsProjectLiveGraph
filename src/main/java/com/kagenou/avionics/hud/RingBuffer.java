package com.kagenou.avionics.hud;

/**
 * Fixed-capacity circular buffer of timestamped float samples for one plot
 * channel. Oldest samples are overwritten once full. Indexing via {@link #timeAt}
 * / {@link #valueAt} runs oldest-to-newest regardless of the internal head.
 */
public final class RingBuffer {
    private final long[] t;
    private final float[] v;
    private final int cap;
    private int head = 0;   // next write position
    private int count = 0;

    public RingBuffer(int capacity) {
        cap = Math.max(2, capacity);
        t = new long[cap];
        v = new float[cap];
    }

    public void push(long time, float value) {
        t[head] = time;
        v[head] = value;
        head = (head + 1) % cap;
        if (count < cap) {
            count++;
        }
    }

    public int size() {
        return count;
    }

    public long timeAt(int i) {
        return t[idx(i)];
    }

    public float valueAt(int i) {
        return v[idx(i)];
    }

    private int idx(int i) {
        int start = (head - count + cap) % cap;
        return (start + i) % cap;
    }
}
