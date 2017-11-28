package org.firstinspires.ftc.teamcode.Components.GliphGrabber;

/**
 * Created by Tyler on 11/27/17.
 *
 * Interface to map and set power to gliph grabber
 */

public interface GliphGrabber {

    double HAND_STOPPED = 0;
    double HAND_OPEN = 1;
    double HAND_CLOSED = -1;

    /**
     * Maps DriveTrain to Phones
     */
    void initHardware();

    /**
     * Set Power to DriveTrain
     */
    void runHardware();
}
/**
 *Tyler rocks
 */