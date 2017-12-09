package org.firstinspires.ftc.teamcode.Components.RelicRecovery;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.ComponentInterface.ComponentHardware;
import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by Tyler on 11/27/17.
 *
 * The Glyph Grabber component is used to retrieve the glyphs
 * and place them in the Cryptoboxes
 */

public class GlyphGrabber extends ComponentHardware {
    // ------------------------- Constants --------------------------
    /**
     * value of when the glyph grabber hand is stopped
     */
    public static final double HAND_STOPPED = 0;
    /**
     * value of when the glyph grabber hand is opening
     */
    public static final double HAND_OPEN = 1;
    /**
     * value of when the glyph grabber hand is retracting
     */
    public static final double HAND_CLOSED = -1;
    // ---------------------- Hardware Objects ----------------------
    /**
     * lift motor
     */
    public DcMotor mLift;
    /**
     * hand servo
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
     * Constructs lift motor and hand servo
     *
     * @param map object that is used to map the glyph grabber
     * @param setRobot object that is used to set the power to the glyph grabber
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
     * maps lift motor and hand servo
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

    @Override
    public void stopHardware() {
       liftPower = 0;
       crHandPosition = HAND_STOPPED;
        setRobot.power(mLift,liftPower,"lift motor");
        setRobot.position(crHand,crHandPosition,"hand crservo");
    }
}