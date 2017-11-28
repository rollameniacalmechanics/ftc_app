package org.firstinspires.ftc.teamcode.Components.DriveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.ComponentHardware;
import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by spmce on 11/26/2017.
 *
 * This drive train is used for robots with a two motor drive train
 */
public class StandardDrive extends ComponentHardware {
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
     * Constructs a two motor drive train
     *
     * @param map object that is used to map the drive train
     * @param setRobot object that is used to set the power to the drive train
     */
    public StandardDrive(Map map, SetRobot setRobot) {
        this.map = map;
        this.setRobot = setRobot;
        mLeft    = null;
        mRight   = null;
        leftPower  = 0;
        rightPower = 0;
    }
    // -------------------------- Mapping ---------------------------
    /**
     * Maps left and right motor of drive train
     */
    @Override
    public void initHardware() {
        mLeft  = map.revMotor("l");
        mRight = map.motor("r");
    }
    // --------------------- Set Hardware Power ---------------------
    /**
     * Sets power to right and left motor of drive train
     */
    @Override
    public void runHardware() {
        setRobot.power(mLeft,leftPower,"left motor");
        setRobot.power(mRight,rightPower,"right motor");
    }
}
