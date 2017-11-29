package org.firstinspires.ftc.teamcode.Components.RelicRecovery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.ComponentInterface.ComponentHardware;
import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by jeppe on 27-11-2017.
 *
 * (description here)
 */

public class RelicRetriever extends ComponentHardware {
    // ------------------------- Constants --------------------------
    public static final double ARM_IN = 0;
    public static final double GRABBER_OPEN = .3;
    public static final double GRABBER_CLOSED = 0;
    // ---------------------- Hardware Objects ----------------------
    /**
     * Lift arm motor
     */
    public DcMotor mArmLift;
    /**
     * Arm servo
     */
    public Servo ssArm;
    /**
     * Relic grabber servo
     */
    public Servo ssRelicGrabber;
    // --------------------- Motor Values ---------------------------
    /**
     * The power for the armlifter
     */
    public double armLiftPower;
    /**
     * the position for the arm
     */
    public double armPosition;
    /**
     * the position for the relic grabber
     */
    public double grabberPosition;
    // ------------------------ Constructor -------------------------
    /**
     * Constructor with hardwaremap and position/power
     *
     * Initializes objects to null and variables to 0
     * 
     * @param map Used to map the objects
     * @param setRobot Used to set the position or power
     */
    public RelicRetriever(Map map, SetRobot setRobot) {
        this.map = map;
        this.setRobot = setRobot;
        mArmLift       = null;
        ssArm          = null;
        ssRelicGrabber = null;
        armLiftPower    = 0;
        armPosition     = ARM_IN;
        grabberPosition = GRABBER_CLOSED;
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
