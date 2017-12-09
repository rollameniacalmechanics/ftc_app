package org.firstinspires.ftc.teamcode.Components.RelicRecovery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.ComponentInterface.ComponentHardware;
import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by Jeppe on 27-11-2017.
 *
 * The Relic Grabber component is used to retrieve the relic and place them on the scoring zones.
 */
public class RelicRetriever extends ComponentHardware {
    // ------------------------- Constants --------------------------
    /**
     *value of when the arm extender is contracted
     */
    public static final double ARM_IN = 0;
    /**
     * value of when the grabber is open
     */
    public static final double GRABBER_OPEN = .3;
    /**
     *value of when the grabber is closed
     */
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
    // -------------------------- Mapping ---------------------------
    /**
     * finds hardware on phone
     */
    @Override
    public void initHardware() {
        mArmLift = map.motor("armLift");
        ssArm = map.revServo("sArm", armPosition);
        ssRelicGrabber = map.servo("sGrabber", grabberPosition);
    }
    // --------------------- Set Hardware Power ---------------------
    /**
     * sets power to hardware
     */
    @Override
    public void runHardware() {
        setRobot.power(mArmLift,armLiftPower,"arm lift motor");
        setRobot.position(ssArm,armPosition,"arm servo");
        setRobot.position(ssRelicGrabber,grabberPosition,"relic grabber servo");
    }

    @Override
    public void stopHardware() {
        armLiftPower    = 0;
        armPosition     = ARM_IN;
        grabberPosition = GRABBER_CLOSED;
        setRobot.power(mArmLift,armLiftPower,"arm lift motor");
        setRobot.position(ssArm,armPosition,"arm servo");
        setRobot.position(ssRelicGrabber,grabberPosition,"relic grabber servo");
    }
}
