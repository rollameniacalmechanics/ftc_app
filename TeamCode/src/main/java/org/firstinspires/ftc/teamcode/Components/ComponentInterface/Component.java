package org.firstinspires.ftc.teamcode.Components.ComponentInterface;

/**
 * Created by spmce on 11/27/2017.
 *
 * Interface to map and set power to components
 */
public interface Component {

    /**
     * Maps Component to Phones
     */
    void initHardware();

    /**
     * Set Power to Component
     */
    void runHardware();
    /**
     * Stops hardware
     */
    void stopHardware();
}
