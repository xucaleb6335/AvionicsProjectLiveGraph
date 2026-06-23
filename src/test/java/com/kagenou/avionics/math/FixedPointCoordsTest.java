package com.kagenou.avionics.math;

import com.kagenou.avionics.math.FixedPointCoords.FP;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Unit tests for the Q8.8 fixed-point coordinate system. */
class FixedPointCoordsTest {

    @Test
    void roundTripsRepresentableValues() {
        // Multiples of the 1/256 step survive quantization exactly.
        for (float v : new float[]{0f, 0.25f, 1f, -3.5f, 100.5f}) {
            assertEquals(v, FixedPointCoords.toFloat(FixedPointCoords.fromFloat(v)), 1e-4f);
        }
    }

    @Test
    void quantizesToTheStep() {
        // 0.001 m is well under half a step (1/512) -> rounds to 0.
        assertEquals(0f, FixedPointCoords.toFloat(FixedPointCoords.fromFloat(0.001f)), 1e-6f);
        assertEquals(FixedPointCoords.STEP, 1f / 256f, 1e-9f);
    }

    @Test
    void clampsToWorldRange() {
        assertEquals(FixedPointCoords.WORLD_MAX,
                FixedPointCoords.toFloat(FixedPointCoords.fromFloat(1000f)), 1e-4f);
        assertEquals(FixedPointCoords.WORLD_MIN,
                FixedPointCoords.toFloat(FixedPointCoords.fromFloat(-1000f)), 1e-4f);
        assertEquals(-128.0f, FixedPointCoords.WORLD_MIN, 1e-4f);
        assertEquals(127.99609375f, FixedPointCoords.WORLD_MAX, 1e-4f);
    }

    @Test
    void shiftHelpersMatchScaling() {
        assertEquals(5f, FixedPointCoords.toFloat(FixedPointCoords.fromInt(5)), 1e-4f);
        assertEquals(5, FixedPointCoords.floorMetres(FixedPointCoords.fromFloat(5.75f)));
        assertEquals(0, FixedPointCoords.ORIGIN.raw());
        assertEquals(0f, FixedPointCoords.ORIGIN.toFloat(), 1e-6f);
    }

    @Test
    void fpArithmeticClampsToRange() {
        FP near = FP.fromFloat(127f);
        assertEquals(FixedPointCoords.WORLD_MAX, near.addMetres(50f).toFloat(), 1e-4f);
    }

    @Test
    void soaStoresAndRetrievesPoints() {
        FixedPointCoords world = new FixedPointCoords(2);
        assertEquals(0, world.add(1f, 2f, 3f));
        assertEquals(1, world.add(-4f, 5.5f, 0f));
        assertEquals(-1, world.add(9f, 9f, 9f)); // full
        assertEquals(2, world.size());
        assertEquals(1f, world.xAt(0), FixedPointCoords.STEP);
        assertEquals(5.5f, world.yAt(1), FixedPointCoords.STEP);
        world.clear();
        assertEquals(0, world.size());
    }
}
