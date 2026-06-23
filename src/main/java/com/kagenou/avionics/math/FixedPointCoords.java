package com.kagenou.avionics.math;

/**
 * Q8.8 signed fixed-point coordinate system (8 integer bits including sign, 8
 * fractional bits). Raw values are 16-bit two's-complement in {@code [-32768, 32767]}
 * and back exactly into a {@code short}; the representable world range is
 * {@code [-128.0, +127.996] m} at a fixed 1/256 m (~3.9 mm) step.
 *
 * <p>Bulk world data is held as a flat Structure-of-Arrays ({@code short[]} per
 * axis) with no nested object graph, so the backing buffers are contiguous and can
 * be handed to native / CUDA kernels directly. The {@link FP} record is a scalar
 * primitive for one-off coordinates (e.g. the drone position).
 */
public final class FixedPointCoords {

    // --- Q8.2 format ---
    public static final int FRAC_BITS = 8;
    public static final int ONE = 1 << FRAC_BITS;                 // 256 raw units == 1.0 m
    public static final float STEP = 1f / ONE;                    // 1/256 m (~3.9 mm) resolution
    public static final short RAW_MIN = Short.MIN_VALUE;          // -32768
    public static final short RAW_MAX = Short.MAX_VALUE;          //  32767
    public static final float WORLD_MIN = RAW_MIN / (float) ONE;  // -128.0 m
    public static final float WORLD_MAX = RAW_MAX / (float) ONE;  // +127.996 m

    /** Quantize metres to Q8.2 raw: scale by 2^FRAC_BITS, round to nearest step, clamp. */
    public static short fromFloat(float metres) {
        return (short) clampRaw(Math.round(metres * ONE));
    }

    /** Dequantize Q8.2 raw to metres. */
    public static float toFloat(short raw) {
        return raw / (float) ONE;
    }

    /** Integer-metre part via arithmetic right shift (drops the 2 fractional bits). */
    public static int floorMetres(short raw) {
        return raw >> FRAC_BITS;
    }

    /** Whole metres to raw via left shift (exact; no rounding needed). */
    public static short fromInt(int metres) {
        return (short) clampRaw(metres << FRAC_BITS);
    }

    public static int clampRaw(int raw) {
        return raw < RAW_MIN ? RAW_MIN : Math.min(raw, RAW_MAX);
    }

    /** Clamp a value in metres to the representable Q8.2 world range. */
    public static float clampWorld(float metres) {
        return metres < WORLD_MIN ? WORLD_MIN : Math.min(metres, WORLD_MAX);
    }

    /**
     * Immutable scalar primitive wrapping a single Q8.2 raw value. Arithmetic stays
     * in fixed point and re-clamps to the world range on every operation.
     */
    public record FP(short raw) {
        public static FP fromFloat(float metres) {
            return new FP(FixedPointCoords.fromFloat(metres));
        }

        public float toFloat() {
            return FixedPointCoords.toFloat(raw);
        }

        /** Add a delta expressed in raw Q8.2 units (e.g. a quantized velocity step). */
        public FP addRaw(int dRaw) {
            return new FP((short) clampRaw(raw + dRaw));
        }

        /** Add a delta expressed in metres. */
        public FP addMetres(float dm) {
            return new FP((short) clampRaw(raw + Math.round(dm * ONE)));
        }

        @Override
        public String toString() {
            return String.format("%.2f", toFloat());
        }
    }

    /** Drone spawn point: 0.0 m on every axis maps to raw (0, 0, 0). */
    public static final FP ORIGIN = new FP((short) 0);

    // --- SoA world storage: flat, contiguous, CUDA-friendly ---
    public final short[] xPos;
    public final short[] yPos;
    public final short[] zPos;
    private final int capacity;
    private int size;

    public FixedPointCoords(int capacity) {
        this.capacity = capacity;
        this.xPos = new short[capacity];
        this.yPos = new short[capacity];
        this.zPos = new short[capacity];
    }

    public int size() {
        return size;
    }

    public int capacity() {
        return capacity;
    }

    /** Append a quantized world point; returns its index, or -1 if the buffer is full. */
    public int add(float xm, float ym, float zm) {
        if (size >= capacity) {
            return -1;
        }
        int i = size++;
        xPos[i] = fromFloat(xm);
        yPos[i] = fromFloat(ym);
        zPos[i] = fromFloat(zm);
        return i;
    }

    public void set(int i, float xm, float ym, float zm) {
        xPos[i] = fromFloat(xm);
        yPos[i] = fromFloat(ym);
        zPos[i] = fromFloat(zm);
    }

    public float xAt(int i) {
        return toFloat(xPos[i]);
    }

    public float yAt(int i) {
        return toFloat(yPos[i]);
    }

    public float zAt(int i) {
        return toFloat(zPos[i]);
    }

    public void clear() {
        size = 0;
    }
}
