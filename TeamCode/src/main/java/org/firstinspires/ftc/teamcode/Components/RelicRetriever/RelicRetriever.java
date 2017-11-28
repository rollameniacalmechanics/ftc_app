package org.firstinspires.ftc.teamcode.Components.RelicRetriever;

/**
 * Created by jeppe on 27-11-2017.
 *
 * Interface to map and set power to drive train
 */

public interface RelicRetriever {

    double GRABBER_CLOSED = 0;
    double ARM_IN = 0;
    /**
     * Maps RelicRetriever to Phones
     */
    void initHardware();

    /**
     * Set Power to RelicRetriever
     */
    void runHardware();



}
