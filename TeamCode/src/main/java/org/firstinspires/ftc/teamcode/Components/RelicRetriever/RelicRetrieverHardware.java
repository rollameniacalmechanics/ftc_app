package org.firstinspires.ftc.teamcode.Components.RelicRetriever;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by jeppe on 27-11-2017.
 *
 * Abstract class for drive train Hardware
 * Contains left and right motors and variables
 */

public abstract class RelicRetrieverHardware implements RelicRetriever {
    // ---------------------- Hardware Objects ----------------------
    /**
     * object used to map hardware
     */
    protected Map map;
    /**
     * object used to set drive train power
     */
    protected SetRobot setRobot;
    // ---------------------- Hardware Devices ----------------------
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

    /**
     * Constructor
     *
     * Initializes objects to null and variables to 0
     */
    // ------------------------ Constructor -------------------------
    public RelicRetrieverHardware(){
        mArmLift       = null;
        ssArm          = null;
        ssRelicGrabber = null;
        armLiftPower         = 0;
        armPosition          = 0;
        grabberPosition = 0;
    }
}
