package org.firstinspires.ftc.teamcode.Components.GliphGrabber;

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
public abstract class GliphGrabberHardware implements GliphGrabber {
    // ---------------------- Hardware Objects ----------------------

    Map map;

    SetRobot setRobot;
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
     * constructer / inti objects to null and varibles to 0
     */
    GliphGrabberHardware() {
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