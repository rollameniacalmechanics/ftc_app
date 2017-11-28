package org.firstinspires.ftc.teamcode.Components.GlyphGrabber;

/**
 * Created by Tyler on 11/27/17.
 *
 * Interface to map and set power to glyph grabber
 */

public interface GlyphGrabber {

    double HAND_STOPPED = 0;
    double HAND_OPEN = 1;
    double HAND_CLOSED = -1;

    /**
     * Maps Component to Phones
     */
    void initHardware();

    /**
     * Set Power to Component
     */
    void runHardware();
}
/**
 *Tyler rocks
 */