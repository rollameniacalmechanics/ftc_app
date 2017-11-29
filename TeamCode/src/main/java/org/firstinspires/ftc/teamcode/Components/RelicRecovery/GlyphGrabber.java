package org.firstinspires.ftc.teamcode.Components.RelicRecovery;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.ComponentInterface.ComponentHardware;
import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by Tyler on 11/27/17.
 *
 * (description here)
 */

public class GlyphGrabber extends ComponentHardware {

    public static final double HAND_STOPPED = 0;
    public static final double HAND_OPEN = 1;
    public static final double HAND_CLOSED = -1;

    /**
     *
     * lift motor
     *
     */
    public DcMotor mLift;
    /**
     *
     * hand servo
     *
     */
    public CRServo crHand;


    // --------------------- Hardware Variables ---------------------
    /**
     * this variable is used to set power the lift motor
     */
    public double liftPower;

    /**
     * this variable is used to set power the hand crServo
     */
    public double crHandPosition;
    // ------------------------ Constructor -------------------------
    /**
     * Constructs a one motor and one servo
     *
     * @param map object that is used to map the Gliph GlyphGrabber
     * @param setRobot object that is used to set the power to the Gliph GlyphGrabber
     */
    public GlyphGrabber(Map map, SetRobot setRobot) {
        this.map = map;
        this.setRobot = setRobot;
        mLift       = null;
        crHand      = null;
        liftPower   = 0;
        crHandPosition = HAND_STOPPED;
    }
    // -------------------------- Mapping ---------------------------
    /**
     * maps lift motor and crHand servo
     */
    @Override
    public void initHardware() {
        mLift  = map.motor("lift");
        crHand = map.revCrservo("crHand");
    }
    // --------------------- Set Hardware Power ---------------------
    /**
     * Sets power to lift motor and hand servo of drive train
     */
    @Override
    public void runHardware() {
        setRobot.power(mLift,liftPower,"lift motor");
        setRobot.position(crHand,crHandPosition,"hand crservo");
    }
}
/**
 *Tyler rocks
 *
 * I agree
 */