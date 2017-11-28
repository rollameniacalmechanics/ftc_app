package org.firstinspires.ftc.teamcode.Components.RelicRetriever;

import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by jeppe on 27-11-2017.
 */

public class Retriever extends RelicRetrieverHardware {
    /**
     * Hardware map object
     */
    Map map;
    /**
     * Hardwaremap object
     */
    SetRobot setRobot;
    /**
     * Constructor with hardwaremap and position/power
     *
     * @param map Used to map the objects
     * @param setRobot Used to set the position or power
     */
    public Retriever(Map map, SetRobot setRobot) {
        this.map = map;
        this.setRobot = setRobot;
    }

    @Override
    public void initHardware() {
        /**
         * Hardware map for armlift
         */
        mArmLift = map.motor("armLift");
        /**
         * Hardware map for armposition
         */
        ssArm = map.revServo("sArm", armPosition);
        /**
         * Hardware map for grabber position
         */
        ssRelicGrabber = map.servo("sGrabber", grabberPosition);
    }

    @Override
    public void runHardware() {
        /**
         * Set power or position for armlift
         */
        setRobot.power(mArmLift,armLiftPower,"arm lift motor");
        /**
         * Set power or position for armservo
         */
        setRobot.position(ssArm,armPosition,"arm servo");
        /**
         * Set power or position for relicgrabber
         */
        setRobot.position(ssRelicGrabber,grabberPosition,"relic grabber servo");
    }
}
