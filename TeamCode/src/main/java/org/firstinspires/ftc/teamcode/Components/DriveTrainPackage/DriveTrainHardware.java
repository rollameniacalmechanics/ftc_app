package org.firstinspires.ftc.teamcode.Components.DriveTrainPackage;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by spmce on 11/26/2017.
 *
 * Abstract class for drive train Hardware
 * Contains left and right motors and variables
 */
public abstract class DriveTrainHardware implements DriveTrain {
    // ---------------------- Hardware Objects ----------------------
    /**
     * object used to map hardware
     */
    Map map;
    /**
     * object used to set drive train power
     */
    SetRobot setRobot;
    // ---------------------- Hardware Devices ----------------------
    /**
     * left motor
     */
    public DcMotor mLeft;
    /**
     * right motor
     */
    public DcMotor mRight;
    // --------------------- Hardware Variables ---------------------
    /**
     * this variable is used to set power the left motor
     */
    public double leftPower;
    /**
     * this variable is used to set power the right motor
     */
    public double rightPower;


    // ------------------------ Constructor -------------------------
    /**
     * constructer / inti objects to null and varibles to 0
     */
    DriveTrainHardware() {
        map      = null;
        setRobot = null;
        mLeft    = null;
        mRight   = null;
        leftPower  = 0;
        rightPower = 0;
    }
}
