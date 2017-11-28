package org.firstinspires.ftc.teamcode.Components.GlyphGrabber;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by Tyler on 11/27/17.
 *
 * Abstract class for drive train Hardware
 * Contains left and right motors and variables
 */
public abstract class GlyphGrabberHardware implements GlyphGrabber {
    // ---------------------- Hardware Objects ----------------------
    /**
     * object used to map hardware
     */
    protected Map map;
    /**
     * object used to set Glyph Grabber power
     */
    protected SetRobot setRobot;
    // ---------------------- Hardware Devices ----------------------
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
     * constructor / inti objects to null and variables to 0
     */
    GlyphGrabberHardware() {
        map         = null;
        setRobot    = null;
        mLift       = null;
        crHand      = null;
        liftPower   = 0;
        crHandPosition = HAND_STOPPED;
    }
}
/**
 *Tyler rocks
 */