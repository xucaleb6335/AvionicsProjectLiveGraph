package com.kagenou.avionics.scene;

import static org.lwjgl.opengl.GL11.*;

/**
 * A quadcopter assembled from {@link Box} and {@link Cylinder} primitives:
 * central body, four arms in an X layout, a motor at each arm end, and a spinning
 * two-blade prop above each motor.
 *
 * <p>Orientation is made unmistakable the same way the old cube used per-face
 * colors: the two <b>front</b> arms (pointing toward {@code -Z}) are red, the rear
 * arms are light gray, and a small red nose block sits at the front of the body.
 * Drawn at the origin; the render loop applies the IMU attitude around it.
 */
public final class Drone {
    private static final float BODY_W = 0.36f, BODY_H = 0.10f, BODY_D = 0.36f;
    private static final float ARM_REACH = 0.46f, ARM_THK = 0.045f;
    private static final float MOTOR_R = 0.055f, MOTOR_H = 0.07f;
    private static final float PROP_LEN = 0.30f, PROP_THK = 0.012f, PROP_W = 0.035f;
    private static final int CYL_SEG = 16;

    // Diagonal headings (degrees about Y) for the four arms. Rotating +X by these
    // angles, headings < 180 land on -Z = the front pair.
    private static final float[] ARM_HEADINGS = {45f, 135f, 225f, 315f};

    private static final float PROP_DEG_PER_SEC = 720f; // 2 rev/s, purely cosmetic

    public void draw(double timeSeconds) {
        drawBody();

        float spin = (float) (timeSeconds * PROP_DEG_PER_SEC);
        for (int i = 0; i < ARM_HEADINGS.length; i++) {
            float heading = ARM_HEADINGS[i];
            boolean front = heading < 180f;
            float propDir = (i % 2 == 0) ? 1f : -1f; // adjacent props counter-rotate

            glPushMatrix();
            glRotatef(heading, 0, 1, 0);
            drawArm(front);
            drawMotorAndProp(spin * propDir);
            glPopMatrix();
        }
    }

    private void drawBody() {
        glColor3f(0.18f, 0.19f, 0.22f);
        Box.draw(BODY_W, BODY_H, BODY_D);

        // Red nose block at the front (-Z) for an at-a-glance heading cue.
        glColor3f(0.90f, 0.15f, 0.15f);
        glPushMatrix();
        glTranslatef(0f, 0f, -BODY_D * 0.5f - 0.03f);
        Box.draw(0.10f, BODY_H * 0.7f, 0.07f);
        glPopMatrix();
    }

    /** Draws one arm extending along local +X from the body. */
    private void drawArm(boolean front) {
        if (front) {
            glColor3f(0.85f, 0.13f, 0.13f); // front: red
        } else {
            glColor3f(0.78f, 0.80f, 0.84f); // rear: light gray
        }
        glPushMatrix();
        glTranslatef(ARM_REACH * 0.5f, 0f, 0f);
        Box.draw(ARM_REACH, ARM_THK, ARM_THK);
        glPopMatrix();
    }

    /** Draws the motor at the arm end and its spinning prop, rotated by {@code spinDeg}. */
    private void drawMotorAndProp(float spinDeg) {
        glPushMatrix();
        glTranslatef(ARM_REACH, MOTOR_H * 0.5f, 0f);

        glColor3f(0.12f, 0.12f, 0.14f);
        Cylinder.draw(MOTOR_R, MOTOR_H, CYL_SEG);

        // Prop sits just above the motor.
        glTranslatef(0f, MOTOR_H * 0.5f + PROP_THK, 0f);
        glRotatef(spinDeg, 0, 1, 0);
        drawProp();

        glPopMatrix();
    }

    private void drawProp() {
        glColor3f(0.75f, 0.78f, 0.85f);
        Box.draw(PROP_LEN, PROP_THK, PROP_W);
        glPushMatrix();
        glRotatef(90, 0, 1, 0);
        Box.draw(PROP_LEN, PROP_THK, PROP_W);
        glPopMatrix();

        glColor3f(0.10f, 0.10f, 0.12f); // hub
        Cylinder.draw(MOTOR_R * 0.4f, PROP_THK * 2f, 10);
    }
}
