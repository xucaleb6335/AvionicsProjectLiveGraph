package com.kagenou.avionics.io;

/**
 * A producer of attitude samples that feeds an {@link AttitudeState}. Implemented
 * by {@link SerialReader} (live hardware) and {@link MockSource} (synthetic feed
 * for development without a board). Lets the app pick a feed at startup and treat
 * both the same way.
 */
public interface AttitudeSource {
    void start();

    void stop();

    /** Short human-readable description for logging, e.g. {@code "serial COM9 @115200"}. */
    String describe();
}
